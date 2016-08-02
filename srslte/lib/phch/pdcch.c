/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 * Copyright 2016 IMDEA Networks Institute
 *
 * \section LICENSE
 *
 * This file is part of OWL, which extends the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "srslte/phch/dci.h"
#include "srslte/phch/regs.h"
#include "srslte/phch/pdcch.h"
#include "srslte/common/phy_common.h"
#include "srslte/utils/bit.h"
#include "srslte/utils/vector.h"
#include "srslte/utils/debug.h"

#define PDCCH_NOF_FORMATS               4
#define PDCCH_FORMAT_NOF_CCE(i)         (1<<i)
#define PDCCH_FORMAT_NOF_REGS(i)        ((1<<i)*9)
#define PDCCH_FORMAT_NOF_BITS(i)        ((1<<i)*72)
#define PWR_THR							1

static void set_cfi(srslte_pdcch_t *q, uint32_t cfi) {
  if (cfi > 0 && cfi < 4) {
    q->nof_regs = (srslte_regs_pdcch_nregs(q->regs, cfi) / 9) * 9;
    q->nof_cce = q->nof_regs / 9;
  } 
}


/** Initializes the PDCCH transmitter and receiver */
int srslte_pdcch_init(srslte_pdcch_t *q, srslte_regs_t *regs, srslte_cell_t cell) {
  int ret = SRSLTE_ERROR_INVALID_INPUTS;
  uint32_t i;

  if (q                         != NULL &&
      regs                      != NULL &&
      srslte_cell_isvalid(&cell)) 
  {   
    ret = SRSLTE_ERROR;
    bzero(q, sizeof(srslte_pdcch_t));
    q->cell = cell;
    q->regs = regs;
    
    /* Allocate memory for the maximum number of PDCCH bits (CFI=3) */
    q->max_bits = (srslte_regs_pdcch_nregs(q->regs, 3) / 9) * 72;

    INFO("Init PDCCH: Max bits: %d, %d ports.\n", 
         q->max_bits, q->cell.nof_ports);

    if (srslte_modem_table_lte(&q->mod, SRSLTE_MOD_QPSK)) {
      goto clean;
    }
    if (srslte_crc_init(&q->crc, SRSLTE_LTE_CRC16, 16)) {
      goto clean;
    }

    for (i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; i++) {
      // we need to pregenerate the sequence for the maximum number of bits, which is 8 times 
      // the maximum number of REGs (for CFI=3)
      if (srslte_sequence_pdcch(&q->seq[i], 2 * i, q->cell.id, 8*srslte_regs_pdcch_nregs(q->regs, 3))) {
        goto clean;
      }
    }

    uint32_t poly[3] = { 0x6D, 0x4F, 0x57 };
    if (srslte_viterbi_init(&q->decoder, SRSLTE_VITERBI_37, poly, SRSLTE_DCI_MAX_BITS + 16, true)) {
      goto clean;
    }

    q->e = srslte_vec_malloc(sizeof(uint8_t) * q->max_bits);
    if (!q->e) {
      goto clean;
    }

    q->llr = srslte_vec_malloc(sizeof(float) * q->max_bits);
    if (!q->llr) {
      goto clean;
    }
    
    bzero(q->llr, sizeof(float) * q->max_bits);

    q->d = srslte_vec_malloc(sizeof(cf_t) * q->max_bits / 2);
    if (!q->d) {
      goto clean;
    }

    for (i = 0; i < SRSLTE_MAX_PORTS; i++) {
      q->ce[i] = srslte_vec_malloc(sizeof(cf_t) * q->max_bits / 2);
      if (!q->ce[i]) {
        goto clean;
      }
      q->x[i] = srslte_vec_malloc(sizeof(cf_t) * q->max_bits / 2);
      if (!q->x[i]) {
        goto clean;
      }
      q->symbols[i] = srslte_vec_malloc(sizeof(cf_t) * q->max_bits / 2);
      if (!q->symbols[i]) {
        goto clean;
      }
    }

    ret = SRSLTE_SUCCESS;
  }
  clean: 
  if (ret == SRSLTE_ERROR) {
    srslte_pdcch_free(q);
  }
  return ret;
}

void srslte_pdcch_free(srslte_pdcch_t *q) {
  int i;

  if (q->e) {
    free(q->e);
  }
  if (q->llr) {
    free(q->llr);
  }
  if (q->d) {
    free(q->d);
  }
  for (i = 0; i < SRSLTE_MAX_PORTS; i++) {
    if (q->ce[i]) {
      free(q->ce[i]);
    }
    if (q->x[i]) {
      free(q->x[i]);
    }
    if (q->symbols[i]) {
      free(q->symbols[i]);
    }
  }

  for (i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; i++) {
    srslte_sequence_free(&q->seq[i]);
  }

  srslte_modem_table_free(&q->mod);
  srslte_viterbi_free(&q->decoder);

  bzero(q, sizeof(srslte_pdcch_t));

}

/** 36.213 v9.1.1 
 * Computes up to max_candidates UE-specific candidates for DCI messages and saves them 
 * in the structure pointed by c.
 * Returns the number of candidates saved in the array c.   
 */
uint32_t srslte_pdcch_ue_locations(srslte_pdcch_t *q, srslte_dci_location_t *c, uint32_t max_candidates,
                        uint32_t nsubframe, uint32_t cfi, uint16_t rnti) {
  
  int l; // this must be int because of the for(;;--) loop
  uint32_t i, k, L, m; 
  uint32_t Yk, ncce;
  const int S[4] = { 6, 12, 8, 16 };

  set_cfi(q, cfi);

  // Compute Yk for this subframe
  Yk = rnti;
  for (m = 0; m < nsubframe+1; m++) {
    Yk = (39827 * Yk) % 65537;
  }

  k = 0;
  // All aggregation levels from 8 to 1
  for (l = 0; l < 3; l++) {
    L = (1 << l);
    // For all possible ncce offset
    for (i = 0; i < SRSLTE_MIN(q->nof_cce / L, S[l]/PDCCH_FORMAT_NOF_CCE(l)); i++) {
      ncce = L * ((Yk + i) % (q->nof_cce / L));      
      if (k                              < max_candidates     &&
          ncce + PDCCH_FORMAT_NOF_CCE(l) <= q->nof_cce) 
      {            
        c[k].L = l;
        c[k].ncce = ncce;
        
        DEBUG("UE-specific SS Candidate %d: nCCE: %d, L: %d\n",
            k, c[k].ncce, c[k].L);            

        k++;          
      } 
    }
  }

  DEBUG("Initiated %d candidate(s) in the UE-specific search space for C-RNTI: 0x%x\n", k, rnti);
  
  return k; 
}



/**
 * 36.213 9.1.1
 * Computes up to max_candidates candidates in the common search space 
 * for DCI messages and saves them in the structure pointed by c.  
 * Returns the number of candidates saved in the array c.   
 */
uint32_t srslte_pdcch_common_locations(srslte_pdcch_t *q, srslte_dci_location_t *c, uint32_t max_candidates, 
                                uint32_t cfi) 
{
  uint32_t i, l, L, k;

  set_cfi(q, cfi);

  k = 0;
  for (l = 3; l > 1; l--) {
    L = (1 << l);
    for (i = 0; i < SRSLTE_MIN(q->nof_cce, 16) / (L); i++) {
      if (k < max_candidates) {
        c[k].L = l;
        c[k].ncce = (L) * (i % (q->nof_cce / (L)));
        DEBUG("Common SS Candidate %d: nCCE: %d, L: %d\n",
            k, c[k].ncce, c[k].L);
        k++;          
      }
    }
  }
  
  INFO("Initiated %d candidate(s) in the Common search space\n", k);
  
  return k;
}

/** 36.213 v9.1.1
 * Computes up to max_candidates UE-specific candidates for DCI messages and saves them
 * in the structure pointed by c.
 * Returns the number of candidates saved in the array c.
 */
uint32_t srslte_pdcch_ue_locations_all(srslte_pdcch_t *q, srslte_dci_location_t *c, uint32_t max_candidates,
                        uint32_t nsubframe, uint32_t cfi) {
	/* IMDEA contribution: obtain all possible location for CCE, without caring about the RNTI */

	uint32_t i, L, k;
	int l;

	set_cfi(q, cfi);

	k = 0;
	for (l = 3; l >= 0; l--) {
		L = (1 << l);
		for (i = 0; i < SRSLTE_MIN(q->nof_cce, 64) / (L); i++) {
			if (k < max_candidates) {
				c[k].L = l;
				c[k].ncce = (L) * (i % (q->nof_cce / (L)));
				c[k].used = 0;
				c[k].checked = 0;
				DEBUG("Common SS Candidate %d: nCCE: %d, L: %d\n",
						k, c[k].ncce, c[k].L);
				k++;
			}
		}
	}

	INFO("Initiated %d candidate(s) out of %d CCEs\n", k,q->nof_cce);

	return k;
}

static void crc_set_mask_rnti(uint8_t *crc, uint16_t rnti) {
  uint32_t i;
  uint8_t mask[16];
  uint8_t *r = mask;

  DEBUG("Mask CRC with RNTI 0x%x\n", rnti);

  srslte_bit_unpack(rnti, &r, 16);
  for (i = 0; i < 16; i++) {
    crc[i] = (crc[i] + mask[i]) % 2;
  }
}

static float dci_decode_and_check_list(srslte_pdcch_t *q, float *e, uint8_t *data, uint32_t E, uint32_t nof_bits, uint16_t *crc, uint8_t *list) {

  uint16_t p_bits, crc_res, c_rnti;
  uint8_t *x;
  uint8_t tmp[3 * (SRSLTE_DCI_MAX_BITS + 16)];
  uint8_t tmp2[10 * (SRSLTE_DCI_MAX_BITS + 16)];
  uint8_t check[(SRSLTE_DCI_MAX_BITS + 16)];
  srslte_convcoder_t encoder;
  int poly[3] = { 0x6D, 0x4F, 0x57 };

  if (q         != NULL         &&
      data      != NULL         &&
      E         <= q->max_bits   &&
      nof_bits  <= SRSLTE_DCI_MAX_BITS) // &&
	  //(nof_bits +16)*3 <= E) // check if it is possbile to store the dci msg in the available cces
  {
	// packet decoding

    bzero(q->rm_f, sizeof(float)*3 * (SRSLTE_DCI_MAX_BITS + 16));

    /* unrate matching */
    srslte_rm_conv_rx(e, E, q->rm_f, 3 * (nof_bits + 16));

    /* viterbi decoder */
    srslte_viterbi_decode_f(&q->decoder, q->rm_f, data, nof_bits + 16);

    if (SRSLTE_VERBOSE_ISDEBUG()) {
      srslte_bit_fprint(stdout, data, nof_bits + 16);
    }

    x = &data[nof_bits];
    p_bits = (uint16_t) srslte_bit_pack(&x, 16);
    crc_res = ((uint16_t) srslte_crc_checksum(&q->crc, data, nof_bits) & 0xffff);
    c_rnti = p_bits ^ crc_res;
    DEBUG("p_bits: 0x%x, crc_checksum: 0x%x, crc_rem: 0x%x\n", p_bits, crc_res,
        c_rnti);

//    printf("op %d crnti %d\n",p_bits ^ crc_res,c_rnti);
    if (list[p_bits ^ crc_res]>0) {
//    	printf("check ok\n");
    	*crc = p_bits ^ crc_res;
    	return 100.0;
    }
    else {
		// re-encoding the packet

		encoder.K = 7;
		encoder.R = 3;
		encoder.tail_biting = true;
		memcpy(encoder.poly, poly, 3 * sizeof(int));

		memcpy(check,data,nof_bits);

		srslte_crc_attach(&q->crc, check, nof_bits);
		crc_set_mask_rnti(&check[nof_bits], c_rnti);

		srslte_convcoder_encode(&encoder, check, tmp, nof_bits + 16);
		srslte_rm_conv_tx(tmp, 3 * (nof_bits + 16), tmp2, E);

		float parcheck = 0.0;
		for (int i=0;i<E;i++) {
			parcheck += ((((e[i]*32+127.5)>127.5)?1:0)==tmp2[i]);
		}
		parcheck = 100*parcheck/E;

		//if (parcheck > 90) printf("nb %d:, p_bits: 0x%x, crc_checksum: 0x%x, crc_rem: 0x%x with prob %.2f\n", nof_bits, p_bits, crc_res, c_rnti, parcheck);
		*crc = p_bits ^ crc_res;
		return parcheck;
    }
  } else {
    fprintf(stderr, "Invalid parameters: E: %d, max_bits: %d, nof_bits: %d\n", E, q->max_bits, nof_bits);
    return -1.0;
  }
}


static float dci_decode_and_check(srslte_pdcch_t *q, float *e, uint8_t *data, uint32_t E, uint32_t nof_bits, uint16_t *crc) {

  uint16_t p_bits, crc_res, c_rnti;
  uint8_t *x;
  uint8_t tmp[3 * (SRSLTE_DCI_MAX_BITS + 16)];
  uint8_t tmp2[10 * (SRSLTE_DCI_MAX_BITS + 16)];
  uint8_t check[(SRSLTE_DCI_MAX_BITS + 16)];
  srslte_convcoder_t encoder;
  int poly[3] = { 0x6D, 0x4F, 0x57 };

  if (q         != NULL         &&
      data      != NULL         &&
      E         <= q->max_bits   &&
      nof_bits  <= SRSLTE_DCI_MAX_BITS) // &&
	  //(nof_bits +16)*3 <= E) // check if it is possbile to store the dci msg in the available cces
  {
	// packet decoding

    bzero(q->rm_f, sizeof(float)*3 * (SRSLTE_DCI_MAX_BITS + 16));

    /* unrate matching */
    srslte_rm_conv_rx(e, E, q->rm_f, 3 * (nof_bits + 16));

    /* viterbi decoder */
    srslte_viterbi_decode_f(&q->decoder, q->rm_f, data, nof_bits + 16);

    if (SRSLTE_VERBOSE_ISDEBUG()) {
      srslte_bit_fprint(stdout, data, nof_bits + 16);
    }

    x = &data[nof_bits];
    p_bits = (uint16_t) srslte_bit_pack(&x, 16);
    crc_res = ((uint16_t) srslte_crc_checksum(&q->crc, data, nof_bits) & 0xffff);
    c_rnti = p_bits ^ crc_res;
    DEBUG("p_bits: 0x%x, crc_checksum: 0x%x, crc_rem: 0x%x\n", p_bits, crc_res,
        c_rnti);

	// re-encoding the packet

	encoder.K = 7;
	encoder.R = 3;
	encoder.tail_biting = true;
	memcpy(encoder.poly, poly, 3 * sizeof(int));

	memcpy(check,data,nof_bits);

	srslte_crc_attach(&q->crc, check, nof_bits);
	crc_set_mask_rnti(&check[nof_bits], c_rnti);

	srslte_convcoder_encode(&encoder, check, tmp, nof_bits + 16);
	srslte_rm_conv_tx(tmp, 3 * (nof_bits + 16), tmp2, E);

	float parcheck = 0.0;
	for (int i=0;i<E;i++) {
		parcheck += ((((e[i]*32+127.5)>127.5)?1:0)==tmp2[i]);
	}
	parcheck = 100*parcheck/E;

	//if (parcheck > 90) printf("nb %d:, p_bits: 0x%x, crc_checksum: 0x%x, crc_rem: 0x%x with prob %.2f\n", nof_bits, p_bits, crc_res, c_rnti, parcheck);
	*crc = p_bits ^ crc_res;
	return parcheck;
  } else {
    fprintf(stderr, "Invalid parameters: E: %d, max_bits: %d, nof_bits: %d\n", E, q->max_bits, nof_bits);
    return -1.0;
  }
}

/** Measure the power on the control channel to decide which locations to decode
 *
 */
float srslte_pdcch_measure_power(srslte_pdcch_t *q,
                            srslte_dci_location_t *location)
							// IMDEA contribution
{
  double mean = 0;
  float ret = 0;

  if (q                 != NULL       &&
      srslte_dci_location_isvalid(location))
  {
    if (location->ncce * 72 + PDCCH_FORMAT_NOF_BITS(location->L) >
      q->nof_cce*72) {
      fprintf(stderr, "Invalid location: nCCE: %d, L: %d, NofCCE: %d\n",
        location->ncce, location->L, q->nof_cce);
    } else {
      uint32_t e_bits = PDCCH_FORMAT_NOF_BITS(location->L);
      for (int i=0;i<e_bits;i++) {
        mean += fabsf(q->llr[location->ncce * 72 + i]);
      }
      mean /= e_bits;

      ret = mean;
    }
  }
  return ret;
}

/** Tries to decode a DCI message from the LLRs stored in the srslte_pdcch_t structure by the function
 * srslte_pdcch_extract_llr(). This function can be called multiple times.
 * The decoded message is stored in msg and the CRC remainder in crc_rem pointer
 *
 */
float srslte_pdcch_check_decode(srslte_pdcch_t *q,
                            srslte_dci_msg_t *msg,
                            srslte_dci_location_t *location,
							uint16_t *brnti)
							// IMDEA contribution
{
  float ret = -1;
  float bprob = -1;
//  int bsize = 0;
  int cnt = 0;
  int max_bits = 0;
  uint16_t crc_rem;
  double mean = 0;
  *brnti = 0;

  if (q                 != NULL       &&
      msg               != NULL       &&
      srslte_dci_location_isvalid(location))
  {
    if (location->ncce * 72 + PDCCH_FORMAT_NOF_BITS(location->L) >
      q->nof_cce*72) {
      fprintf(stderr, "Invalid location: nCCE: %d, L: %d, NofCCE: %d\n",
        location->ncce, location->L, q->nof_cce);
    } else {
      uint32_t e_bits = PDCCH_FORMAT_NOF_BITS(location->L);
      for (int i=0;i<e_bits;i++) {
        mean += fabsf(q->llr[location->ncce * 72 + i]);
      }
      mean /= e_bits;

      uint32_t nof_bits = 27;
      max_bits = 43; // increase to 57 for 20 MHz
      if (mean > PWR_THR) {
    	  	for (int j=nof_bits;j<max_bits;j+=14) {
          		ret = dci_decode_and_check(q, &q->llr[location->ncce * 72], msg->data, e_bits, j, &crc_rem);
          		if (ret >= 0) {
          			if (ret >= 97) cnt++;
          			if (ret > bprob) {
          				bprob = ret;
//          				bsize = j;
          				*brnti = crc_rem;
          				//printf("prob: %f, size %d, crnti %x\n", ret, j, brnti);
          			}
          		}
          	}
      }

    }
  }
  return bprob;
}


/** Tries to decode a DCI message from the LLRs stored in the srslte_pdcch_t structure by the function
 * srslte_pdcch_extract_llr(). This function can be called multiple times.
 * The decoded message is stored in msg and the CRC remainder in crc_rem pointer
 *
 */
int srslte_pdcch_decode_msg_power(srslte_pdcch_t *q,
                            srslte_dci_msg_t *msg,
                            srslte_dci_location_t *location,
							uint32_t sfn,
							uint32_t sf_idx)
							// IMDEA contribution
{
  int ret = -1;
  int bprob = -1;
  int bsize = 0;
  int cnt = 0;
  uint16_t crc_rem;
  double mean = 0;
  uint16_t brnti = 0;
  srslte_ra_dl_dci_t dl_dci_unpacked;
  srslte_ra_ul_dci_t ul_dci_unpacked;
  srslte_ra_dl_grant_t dl_grant;
  srslte_ra_ul_grant_t ul_grant;
  srslte_dci_msg_t dci_tmp;


  if (q                 != NULL       &&
      msg               != NULL       &&
      srslte_dci_location_isvalid(location))
  {
    if (location->ncce * 72 + PDCCH_FORMAT_NOF_BITS(location->L) >
      q->nof_cce*72) {
      fprintf(stderr, "Invalid location: nCCE: %d, L: %d, NofCCE: %d\n",
        location->ncce, location->L, q->nof_cce);
    } else {
      uint32_t e_bits = PDCCH_FORMAT_NOF_BITS(location->L);
      for (int i=0;i<e_bits;i++) {
        mean += fabsf(q->llr[location->ncce * 72 + i]);
      }
      mean /= e_bits;

      uint32_t nof_bits[6][6] = {
    		  {8,19,21,22,28,31},  //1.4 MHz
    		  {10,22,23,25,31,34}, //3 MHz
    		  {12,25,27,36,39,-1}, //5 MHz
    		  {13,27,28,31,41,43}, //10 MHz
    		  {14,27,29,33,42,45}, //15 MHz
    		  {15,28,30,39,48,51}  //20 MHz
      };
      uint8_t bind = 0;
      switch (q->cell.nof_prb) {
      case 6:
    	  bind = 0;
    	  break;
      case 15:
    	  bind = 1;
    	  break;
      case 25:
    	  bind = 2;
    	  break;
      case 50:
    	  bind = 3;
    	  break;
      case 75:
    	  bind = 4;
    	  break;
      case 100:
    	  bind = 5;
    	  break;
      }
      if (mean > PWR_THR) {
    	  	for (int j=0;j<6;j++) {
    	  		if (nof_bits[bind][j]>0) {
					ret = (int) round(dci_decode_and_check(q, &q->llr[location->ncce * 72], dci_tmp.data, e_bits, nof_bits[bind][j], &crc_rem));
					if (ret >= 0) {
						if (ret >= 97) cnt++;
						if (ret > bprob) {
							bprob = ret;
							bsize = nof_bits[bind][j];
							msg->nof_bits = nof_bits[bind][j];
							memcpy(msg->data,dci_tmp.data,msg->nof_bits);
							brnti = crc_rem;
							//printf("prob: %f, size %d, crnti %x\n", ret, j, brnti);
						}
					}
    	  		}
          	}
      }

      if (bprob >= 97 && brnti <= 0xffff) {
    	  printf("%d.%d nCCE: %d, L: %d, power: %f, size %d, prob %d rnti 0x%x (OK)\n", sfn, sf_idx, location->ncce, location->L, mean, bsize, bprob, brnti);
    	  srslte_dci_msg_to_trace(msg, brnti, q->cell.nof_prb, &dl_dci_unpacked, &ul_dci_unpacked, &dl_grant, &ul_grant, sf_idx, sfn, bprob, location->ncce, location->L, q->cell.phich_length, mean);
      } else if (bprob >= 90 && brnti <= 0xffff) {
    	  //if (location->L == 0) printf("%d.%d nCCE: %d, L: %d, power: %f not decoded\n", sfn, sf_idx, location->ncce, location->L, mean);
    	  printf("%d.%d nCCE: %d, L: %d, power: %f, size %d, prob %d rnti 0x%x (MAYBE)\n", sfn, sf_idx, location->ncce, location->L, mean, bsize, bprob, brnti);
    	  srslte_dci_msg_to_trace(msg, brnti, q->cell.nof_prb, &dl_dci_unpacked, &ul_dci_unpacked, &dl_grant, &ul_grant, sf_idx, sfn, bprob, location->ncce, location->L, q->cell.phich_length, mean);
      } else if (bprob >= 0 && brnti <= 0xffff) {
    	  //if (location->L == 0) printf("%d.%d nCCE: %d, L: %d, power: %f not decoded\n", sfn, sf_idx, location->ncce, location->L, mean);
    	  printf("%d.%d nCCE: %d, L: %d, power: %f, size %d, prob %d rnti 0x%x (ERROR)\n", sfn, sf_idx, location->ncce, location->L, mean, bsize, bprob, brnti);
      }
    }
  }
  return cnt;
}







/** 36.212 5.3.3.2 to 5.3.3.4
 *
 * Returns XOR between parity and remainder bits
 *
 * TODO: UE transmit antenna selection CRC mask
 */
static int dci_decode(srslte_pdcch_t *q, float *e, uint8_t *data, uint32_t E, uint32_t nof_bits, uint16_t *crc) {

  uint16_t p_bits, crc_res;
  uint8_t *x;

  if (q         != NULL         &&
      data      != NULL         &&
      E         <= q->max_bits   && 
      nof_bits  <= SRSLTE_DCI_MAX_BITS)
  {
    bzero(q->rm_f, sizeof(float)*3 * (SRSLTE_DCI_MAX_BITS + 16));
    
    /* unrate matching */
    srslte_rm_conv_rx(e, E, q->rm_f, 3 * (nof_bits + 16));
    
    /* viterbi decoder */
    srslte_viterbi_decode_f(&q->decoder, q->rm_f, data, nof_bits + 16);

    x = &data[nof_bits];
    p_bits = (uint16_t) srslte_bit_pack(&x, 16);
    crc_res = ((uint16_t) srslte_crc_checksum(&q->crc, data, nof_bits) & 0xffff);
    
    if (crc) {
      *crc = p_bits ^ crc_res; 
    }
    return SRSLTE_SUCCESS;
  } else {
    fprintf(stderr, "Invalid parameters: E: %d, max_bits: %d, nof_bits: %d\n", E, q->max_bits, nof_bits);
    return SRSLTE_ERROR_INVALID_INPUTS;
  }
}

/** Tries to decode a DCI message from the LLRs stored in the srslte_pdcch_t structure by the function
 * srslte_pdcch_extract_llr(). This function can be called multiple times.
 * The decoded message is stored in msg and the CRC remainder in crc_rem pointer
 *
 */
float srslte_pdcch_decode_msg_check(srslte_pdcch_t *q,
                            srslte_dci_msg_t *msg,
                            srslte_dci_location_t *location,
                            srslte_dci_format_t format,
                            uint16_t *crc_rem,
							uint8_t *list)
							// IMDEA contribution
{
  float ret = -1;
  float mean = 0;

  if (q                 != NULL       &&
      msg               != NULL       &&
      srslte_dci_location_isvalid(location)  &&
      crc_rem           != NULL)
  {
    if (location->ncce * 72 + PDCCH_FORMAT_NOF_BITS(location->L) >
      q->nof_cce*72) {
      fprintf(stderr, "Invalid location: nCCE: %d, L: %d, NofCCE: %d\n",
        location->ncce, location->L, q->nof_cce);
    } else {
      uint32_t nof_bits = srslte_dci_format_all_sizeof_lut(format, q->cell.nof_prb);
      uint32_t e_bits = PDCCH_FORMAT_NOF_BITS(location->L);
      DEBUG("Decoding DCI offset %d, e_bits: %d, msg_len %d (nCCE: %d, L: %d)\n",
            location->ncce * 72, e_bits, nof_bits, location->ncce, location->L);

      mean = 0;
      for (int i=0;i<e_bits;i++) {
        mean += fabsf(q->llr[location->ncce * 72 + i]);
      }
      mean /= e_bits;

      if (mean > PWR_THR) {
    	ret = dci_decode_and_check_list(q, &q->llr[location->ncce * 72], msg->data, e_bits, nof_bits, crc_rem, list);

//    	printf("nCCE: %d, L: %d, mean power: %f, prob: %f, nof_bits %d\n",location->ncce, location->L, mean, ret, nof_bits);
        msg->nof_bits = nof_bits;
        // decomment the following to print the raw bits
//        for (int i=0; i< msg->nof_bits; i++) {
//		  fprintf(stdout,"%d",msg->data[i]);
//		}
//		fprintf(stdout,"\n");

      } else {
        ret = -1.0;
      }
    }
  }
  return ret;
}

/** Tries to decode a DCI message from the LLRs stored in the srslte_pdcch_t structure by the function 
 * srslte_pdcch_extract_llr(). This function can be called multiple times. 
 * The decoded message is stored in msg and the CRC remainder in crc_rem pointer
 * 
 */
int srslte_pdcch_decode_msg(srslte_pdcch_t *q, 
                            srslte_dci_msg_t *msg, 
                            srslte_dci_location_t *location, 
                            srslte_dci_format_t format, 
                            uint16_t *crc_rem) 
{
  int ret = SRSLTE_ERROR_INVALID_INPUTS;
  if (q                 != NULL       && 
      msg               != NULL       && 
      srslte_dci_location_isvalid(location)  &&
      crc_rem           != NULL)
  {
    if (location->ncce * 72 + PDCCH_FORMAT_NOF_BITS(location->L) > 
      q->nof_cce*72) {
      fprintf(stderr, "Invalid location: nCCE: %d, L: %d, NofCCE: %d\n", 
        location->ncce, location->L, q->nof_cce);
    } else {
      uint32_t nof_bits = srslte_dci_format_sizeof_lut(format, q->cell.nof_prb);
      uint32_t e_bits = PDCCH_FORMAT_NOF_BITS(location->L);
    
      double mean = 0; 
      for (int i=0;i<e_bits;i++) {
        mean += fabsf(q->llr[location->ncce * 72 + i]);
      }
      mean /= e_bits; 
      if (mean > PWR_THR) {
        ret = dci_decode(q, &q->llr[location->ncce * 72], 
                        msg->data, e_bits, nof_bits, crc_rem);
        if (ret == SRSLTE_SUCCESS) {
          msg->nof_bits = nof_bits;
        } 
        if (crc_rem) {
          DEBUG("Decoded DCI: nCCE=%d, L=%d, msg_len=%d, mean=%f, crc_rem=0x%x\n", 
            location->ncce, location->L, nof_bits, mean, *crc_rem);
        }
      } else {
        DEBUG("Skipping DCI:  nCCE=%d, L=%d, msg_len=%d, mean=%f\n",
              location->ncce, location->L, nof_bits, mean);
        ret = SRSLTE_SUCCESS;
      }
    }
  }
  return ret;
}

int cnt=0;

/** Extracts the LLRs from srslte_dci_location_t location of the subframe and stores them in the srslte_pdcch_t structure. 
 * DCI messages can be extracted from this location calling the function srslte_pdcch_decode_msg(). 
 * Every time this function is called (with a different location), the last demodulated symbols are overwritten and
 * new messages from other locations can be decoded 
 */
int srslte_pdcch_extract_llr(srslte_pdcch_t *q, cf_t *sf_symbols, cf_t *ce[SRSLTE_MAX_PORTS], float noise_estimate, 
                      uint32_t nsubframe, uint32_t cfi) {

  int ret = SRSLTE_ERROR_INVALID_INPUTS;
  
  /* Set pointers for layermapping & precoding */
  uint32_t i, nof_symbols;
  cf_t *x[SRSLTE_MAX_LAYERS];

  if (q                 != NULL && 
      nsubframe         <  10   &&
      cfi               >  0    &&
      cfi               <  4)
  {
    set_cfi(q, cfi);
    
    uint32_t e_bits = 72*q->nof_cce;
    nof_symbols = e_bits/2;
    ret = SRSLTE_ERROR;
    bzero(q->llr, sizeof(float) * q->max_bits);
    
    DEBUG("Extracting LLRs: E: %d, SF: %d, CFI: %d\n",
        e_bits, nsubframe, cfi);

    /* number of layers equals number of ports */
    for (i = 0; i < q->cell.nof_ports; i++) {
      x[i] = q->x[i];
    }
    memset(&x[q->cell.nof_ports], 0, sizeof(cf_t*) * (SRSLTE_MAX_LAYERS - q->cell.nof_ports));
          
    /* extract symbols */
    int n = srslte_regs_pdcch_get(q->regs, sf_symbols, q->symbols[0]);
    if (nof_symbols != n) {
      fprintf(stderr, "Expected %d PDCCH symbols but got %d symbols\n", nof_symbols, n);
      return ret;
    }

    /* extract channel estimates */
    for (i = 0; i < q->cell.nof_ports; i++) {
      n = srslte_regs_pdcch_get(q->regs, ce[i], q->ce[i]);
      if (nof_symbols != n) {
        fprintf(stderr, "Expected %d PDCCH symbols but got %d symbols\n", nof_symbols, n);
        return ret;
      }
    }

    /* in control channels, only diversity is supported */
    if (q->cell.nof_ports == 1) {
      /* no need for layer demapping */
      srslte_predecoding_single(q->symbols[0], q->ce[0], q->d, nof_symbols, noise_estimate);
    } else {
      srslte_predecoding_diversity(q->symbols[0], q->ce, x, q->cell.nof_ports, nof_symbols);
      srslte_layerdemap_diversity(x, q->d, q->cell.nof_ports, nof_symbols / q->cell.nof_ports);
    }

    /* demodulate symbols */
    srslte_demod_soft_demodulate(SRSLTE_MOD_QPSK, q->d, q->llr, nof_symbols);

    /* descramble */
    srslte_scrambling_f_offset(&q->seq[nsubframe], q->llr, 0, e_bits);
    
    ret = SRSLTE_SUCCESS;
  } 
  return ret;  
}



//static void crc_set_mask_rnti(uint8_t *crc, uint16_t rnti) {
//  uint32_t i;
//  uint8_t mask[16];
//  uint8_t *r = mask;
//
//  DEBUG("Mask CRC with RNTI 0x%x\n", rnti);
//
//  srslte_bit_unpack(rnti, &r, 16);
//  for (i = 0; i < 16; i++) {
//    crc[i] = (crc[i] + mask[i]) % 2;
//  }
//}

/** 36.212 5.3.3.2 to 5.3.3.4
 * TODO: UE transmit antenna selection CRC mask
 */
static int dci_encode(srslte_pdcch_t *q, uint8_t *data, uint8_t *e, uint32_t nof_bits, uint32_t E,
    uint16_t rnti) {
  srslte_convcoder_t encoder;
  uint8_t tmp[3 * (SRSLTE_DCI_MAX_BITS + 16)];
  
  if (q                 != NULL        && 
      data              != NULL        && 
      e                 != NULL        && 
      nof_bits          < SRSLTE_DCI_MAX_BITS &&
      E                 < q->max_bits)
  {

    int poly[3] = { 0x6D, 0x4F, 0x57 };
    encoder.K = 7;
    encoder.R = 3;
    encoder.tail_biting = true;
    memcpy(encoder.poly, poly, 3 * sizeof(int));

    srslte_crc_attach(&q->crc, data, nof_bits);
    crc_set_mask_rnti(&data[nof_bits], rnti);

    srslte_convcoder_encode(&encoder, data, tmp, nof_bits + 16);

    DEBUG("CConv output: ", 0);
    if (SRSLTE_VERBOSE_ISDEBUG()) {
      srslte_vec_fprint_b(stdout, tmp, 3 * (nof_bits + 16));
    }

    srslte_rm_conv_tx(tmp, 3 * (nof_bits + 16), e, E);
    
    return SRSLTE_SUCCESS;
  } else {
    return SRSLTE_ERROR_INVALID_INPUTS;
  }
}

/** Encodes ONE DCI message and allocates the encoded bits to the srslte_dci_location_t indicated by 
 * the parameter location. The CRC is scrambled with the RNTI parameter. 
 * This function can be called multiple times and encoded DCI messages will be allocated to the 
 * sf_symbols buffer ready for transmission. 
 * If the same location is provided in multiple messages, the encoded bits will be overwritten. 
 * 
 * @TODO: Use a bitmask and CFI to ensure message locations are valid and old messages are not overwritten. 
 */
int srslte_pdcch_encode(srslte_pdcch_t *q, srslte_dci_msg_t *msg, srslte_dci_location_t location, uint16_t rnti, 
                 cf_t *sf_symbols[SRSLTE_MAX_PORTS], uint32_t nsubframe, uint32_t cfi) 
{

  int ret = SRSLTE_ERROR_INVALID_INPUTS;
  uint32_t i;
  cf_t *x[SRSLTE_MAX_LAYERS];
  uint32_t nof_symbols;
  
  if (q                 != NULL &&
      sf_symbols        != NULL && 
      nsubframe         <  10   &&
      cfi               >  0    &&
      cfi               <  4    && 
      srslte_dci_location_isvalid(&location))
  {

    set_cfi(q, cfi);

    uint32_t e_bits = PDCCH_FORMAT_NOF_BITS(location.L);
    nof_symbols = e_bits/2;
    ret = SRSLTE_ERROR;
    
    if (location.ncce + PDCCH_FORMAT_NOF_CCE(location.L) <= q->nof_cce && 
        msg->nof_bits < SRSLTE_DCI_MAX_BITS) 
    {      
      DEBUG("Encoding DCI: Nbits: %d, E: %d, nCCE: %d, L: %d, RNTI: 0x%x\n",
          msg->nof_bits, e_bits, location.ncce, location.L, rnti);

      dci_encode(q, msg->data, q->e, msg->nof_bits, e_bits, rnti);
    
      /* number of layers equals number of ports */
      for (i = 0; i < q->cell.nof_ports; i++) {
        x[i] = q->x[i];
      }
      memset(&x[q->cell.nof_ports], 0, sizeof(cf_t*) * (SRSLTE_MAX_LAYERS - q->cell.nof_ports));

      srslte_scrambling_b_offset(&q->seq[nsubframe], q->e, 72 * location.ncce, e_bits);
      
      DEBUG("Scrambling output: ", 0);
      if (SRSLTE_VERBOSE_ISDEBUG()) {        
        srslte_vec_fprint_b(stdout, q->e, e_bits);
      }
      
      srslte_mod_modulate(&q->mod, q->e, q->d, e_bits);

      /* layer mapping & precoding */
      if (q->cell.nof_ports > 1) {
        srslte_layermap_diversity(q->d, x, q->cell.nof_ports, nof_symbols);
        srslte_precoding_diversity(x, q->symbols, q->cell.nof_ports, nof_symbols / q->cell.nof_ports);
      } else {
        memcpy(q->symbols[0], q->d, nof_symbols * sizeof(cf_t));
      }
      
      /* mapping to resource elements */
      for (i = 0; i < q->cell.nof_ports; i++) {
        srslte_regs_pdcch_put_offset(q->regs, q->symbols[i], sf_symbols[i], 
                              location.ncce * 9, PDCCH_FORMAT_NOF_REGS(location.L));
      }
      
      ret = SRSLTE_SUCCESS;
      
    } else {
        fprintf(stderr, "Illegal DCI message nCCE: %d, L: %d, nof_cce: %d\n", location.ncce, location.L, q->nof_cce);
    }
  } 
  return ret;
}

