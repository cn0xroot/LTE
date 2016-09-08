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

#include "srslte/ue/ue_dl.h"

#include <complex.h>
#include <math.h>
#include <string.h>

//#define POWER_ONLY
#define PROB_THR 97

#define CURRENT_FFTSIZE   srslte_symbol_sz(q->cell.nof_prb)
#define CURRENT_SFLEN     SRSLTE_SF_LEN(CURRENT_FFTSIZE)

#define CURRENT_SLOTLEN_RE SRSLTE_SLOT_LEN_RE(q->cell.nof_prb, q->cell.cp)
#define CURRENT_SFLEN_RE SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)

#define MAX_CANDIDATES 80

#define RNTILIFELEN 2

static srslte_dci_format_t ue_formats[] = {SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1}; // Only TM1 and TM2 are currently supported 
const uint32_t nof_ue_formats = 2; 

static srslte_dci_format_t common_formats[] = {SRSLTE_DCI_FORMAT1A,SRSLTE_DCI_FORMAT1C};
const uint32_t nof_common_formats = 2; 

//srslte_dci_format_t ue_all_formats[] = {  SRSLTE_DCI_FORMAT0, SRSLTE_DCI_FORMAT1, SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1B, SRSLTE_DCI_FORMAT1C, SRSLTE_DCI_FORMAT1D, SRSLTE_DCI_FORMAT2, SRSLTE_DCI_FORMAT2A};
//const uint32_t nof_ue_all_formats = 8;

srslte_dci_format_t ue_all_formats[] = {  SRSLTE_DCI_FORMAT0, SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1C, SRSLTE_DCI_FORMAT2A};
const uint32_t nof_ue_all_formats = 4;

int srslte_ue_dl_init(srslte_ue_dl_t *q, 
               srslte_cell_t cell) 
{
  int ret = SRSLTE_ERROR_INVALID_INPUTS; 
  
  if (q                 != NULL &&
      srslte_cell_isvalid(&cell))   
  {
    ret = SRSLTE_ERROR;
    
    bzero(q, sizeof(srslte_ue_dl_t));
    
    q->cell = cell; 
    q->pkt_errors = 0;
    q->pkts_total = 0;
    q->pending_ul_dci_rnti = 0; 
    q->sample_offset = 0; 
    
    if (srslte_ofdm_rx_init(&q->fft, q->cell.cp, q->cell.nof_prb)) {
      fprintf(stderr, "Error initiating FFT\n");
      goto clean_exit;
    }
    if (srslte_chest_dl_init(&q->chest, cell)) {
      fprintf(stderr, "Error initiating channel estimator\n");
      goto clean_exit;
    }
    if (srslte_regs_init(&q->regs, q->cell)) {
      fprintf(stderr, "Error initiating REGs\n");
      goto clean_exit;
    }
    if (srslte_pcfich_init(&q->pcfich, &q->regs, q->cell)) {
      fprintf(stderr, "Error creating PCFICH object\n");
      goto clean_exit;
    }
    if (srslte_phich_init(&q->phich, &q->regs, q->cell)) {
      fprintf(stderr, "Error creating PHICH object\n");
      goto clean_exit;
    }

    if (srslte_pdcch_init(&q->pdcch, &q->regs, q->cell)) {
      fprintf(stderr, "Error creating PDCCH object\n");
      goto clean_exit;
    }

    if (srslte_pdsch_init(&q->pdsch, q->cell)) {
      fprintf(stderr, "Error creating PDSCH object\n");
      goto clean_exit;
    }
    if (srslte_softbuffer_rx_init(&q->softbuffer, q->cell.nof_prb)) {
      fprintf(stderr, "Error initiating soft buffer\n");
      goto clean_exit;
    }
    if (srslte_cfo_init(&q->sfo_correct, q->cell.nof_prb*SRSLTE_NRE)) {
      fprintf(stderr, "Error initiating SFO correct\n");
      goto clean_exit;
    }
    srslte_cfo_set_tol(&q->sfo_correct, 1e-5/q->fft.symbol_sz);
    
    q->sf_symbols = srslte_vec_malloc(CURRENT_SFLEN_RE * sizeof(cf_t));
    if (!q->sf_symbols) {
      perror("malloc");
      goto clean_exit; 
    }
    for (uint32_t i=0;i<q->cell.nof_ports;i++) {
      q->ce[i] = srslte_vec_malloc(CURRENT_SFLEN_RE * sizeof(cf_t));
      if (!q->ce[i]) {
        perror("malloc");
        goto clean_exit; 
      }
    }
    
    srslte_ue_dl_reset_rnti_list(q);

    ret = SRSLTE_SUCCESS;
  } else {
    fprintf(stderr, "Invalid cell properties: Id=%d, Ports=%d, PRBs=%d\n",
            cell.id, cell.nof_ports, cell.nof_prb);      
  }

clean_exit: 
  if (ret == SRSLTE_ERROR) {
    srslte_ue_dl_free(q);
  }
  return ret;
}

void srslte_ue_dl_free(srslte_ue_dl_t *q) {
  if (q) {
    srslte_ofdm_rx_free(&q->fft);
    srslte_chest_dl_free(&q->chest);
    srslte_regs_free(&q->regs);
    srslte_pcfich_free(&q->pcfich);
    srslte_phich_free(&q->phich);
    srslte_pdcch_free(&q->pdcch);
    srslte_pdsch_free(&q->pdsch);
    srslte_cfo_free(&q->sfo_correct);
    srslte_softbuffer_rx_free(&q->softbuffer);
    if (q->sf_symbols) {
      free(q->sf_symbols);
    }
    for (uint32_t i=0;i<q->cell.nof_ports;i++) {
      if (q->ce[i]) {
        free(q->ce[i]);
      }
    }
    bzero(q, sizeof(srslte_ue_dl_t));
  }
}

void srslte_ue_dl_reset_rnti_list(srslte_ue_dl_t *q) {
	bzero(q->rnti_list, 65536*sizeof(uint8_t));
	bzero(q->rnti_cnt, 65536*sizeof(uint8_t));
	for (int i = 1; i <= 10; i++) {
		q->rnti_list[i] = 1;
	}
	q->rnti_list[65534] = 1;
	q->rnti_list[65535] = 1;
}

void srslte_ue_dl_reset_rnti_user(srslte_ue_dl_t *q, uint16_t user) {
	q->rnti_list[user] = RNTILIFELEN;
	q->rnti_cnt[user]++;
}

void srslte_ue_dl_reset_rnti_user_to(srslte_ue_dl_t *q, uint16_t user, uint16_t val) {
	q->rnti_list[user] = val;
}

void srslte_ue_dl_update_rnti_list(srslte_ue_dl_t *q) {
	for (int i = 10; i < 65533; i++) {
		q->rnti_list[i] = (q->rnti_list[i]>0) ? q->rnti_list[i]-1 : 0;
	}
}

int rnti_in_list(srslte_ue_dl_t *q, uint16_t check) {
	return (q->rnti_list[check]);
}

/* Precalculate the PDSCH scramble sequences for a given RNTI. This function takes a while 
 * to execute, so shall be called once the final C-RNTI has been allocated for the session.
 * For the connection procedure, use srslte_pusch_encode_rnti() or srslte_pusch_decode_rnti() functions 
 */
void srslte_ue_dl_set_rnti(srslte_ue_dl_t *q, uint16_t rnti) {
  srslte_pdsch_set_rnti(&q->pdsch, rnti);
  q->current_rnti = rnti; 
}

void srslte_ue_dl_reset(srslte_ue_dl_t *q) {
  srslte_softbuffer_rx_reset(&q->softbuffer);
  bzero(&q->pdsch_cfg, sizeof(srslte_pdsch_cfg_t));
}

void srslte_ue_dl_set_sample_offset(srslte_ue_dl_t * q, float sample_offset) {
  q->sample_offset = sample_offset; 
}

/** Applies the following operations to a subframe of synchronized samples: 
 *    - OFDM demodulation
 *    - Channel estimation 
 *    - PCFICH decoding
 *    - PDCCH decoding: Find DCI for RNTI given by previous call to srslte_ue_dl_set_rnti()
 *    - PDSCH decoding: Decode TB scrambling with RNTI given by srslte_ue_dl_set_rnti()
 */
int srslte_ue_dl_decode(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t sf_idx) {
  return srslte_ue_dl_decode_rnti_rv(q, input, data, sf_idx, q->current_rnti, 0);
}

int srslte_ue_dl_decode_rnti(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t sf_idx, uint16_t rnti) {
  return srslte_ue_dl_decode_rnti_rv(q, input, data, sf_idx, rnti, 0);
}

int srslte_ue_dl_decode_fft_estimate(srslte_ue_dl_t *q, cf_t *input, uint32_t sf_idx, uint32_t *cfi) {
  if (input && q && cfi && sf_idx < SRSLTE_NSUBFRAMES_X_FRAME) {
    
    /* Run FFT for all subframe data */
    srslte_ofdm_rx_sf(&q->fft, input, q->sf_symbols);
    
    /* Correct SFO multiplying by complex exponential in the time domain */
    if (q->sample_offset) {
      for (int i=0;i<2*SRSLTE_CP_NSYMB(q->cell.cp);i++) {
        srslte_cfo_correct(&q->sfo_correct, 
                         &q->sf_symbols[i*q->cell.nof_prb*SRSLTE_NRE], 
                         &q->sf_symbols[i*q->cell.nof_prb*SRSLTE_NRE], 
                         q->sample_offset / q->fft.symbol_sz);
      }
    }
    return srslte_ue_dl_decode_estimate(q, sf_idx, cfi); 
  } else {
    return SRSLTE_ERROR_INVALID_INPUTS; 
  }
}

int srslte_ue_dl_decode_estimate(srslte_ue_dl_t *q, uint32_t sf_idx, uint32_t *cfi) {
  float cfi_corr; 
  if (q && cfi && sf_idx < SRSLTE_NSUBFRAMES_X_FRAME) {
    
    /* Get channel estimates for each port */
    srslte_chest_dl_estimate(&q->chest, q->sf_symbols, q->ce, sf_idx);

    /* First decode PCFICH and obtain CFI */
    if (srslte_pcfich_decode(&q->pcfich, q->sf_symbols, q->ce, 
                             srslte_chest_dl_get_noise_estimate(&q->chest), 
                             sf_idx, cfi, &cfi_corr)<0) {
      fprintf(stderr, "Error decoding PCFICH\n");
      return SRSLTE_ERROR;
    }

    INFO("Decoded CFI=%d with correlation %.2f, sf_idx=%d\n", *cfi, cfi_corr, sf_idx);

    if (srslte_regs_set_cfi(&q->regs, *cfi)) {
      fprintf(stderr, "Error setting CFI\n");
      return SRSLTE_ERROR;
    }
    
    return SRSLTE_SUCCESS; 
  } else {
    return SRSLTE_ERROR_INVALID_INPUTS; 
  }
}


int srslte_ue_dl_cfg_grant(srslte_ue_dl_t *q, srslte_ra_dl_grant_t *grant, uint32_t cfi, uint32_t sf_idx, uint32_t rvidx) 
{
  return srslte_pdsch_cfg(&q->pdsch_cfg, q->cell, grant, cfi, sf_idx, rvidx);
}

int srslte_ue_dl_decode_rnti_rv_packet(srslte_ue_dl_t *q, srslte_ra_dl_grant_t *grant, uint8_t *data, 
                                uint32_t cfi, uint32_t sf_idx, uint16_t rnti, uint32_t rvidx) 
{
  int ret = SRSLTE_ERROR; 

  q->nof_detected++;
  
  /* Setup PDSCH configuration for this CFI, SFIDX and RVIDX */
  if (srslte_ue_dl_cfg_grant(q, grant, cfi, sf_idx, rvidx)) {
    return SRSLTE_ERROR; 
  }
  
  if (q->pdsch_cfg.rv == 0) {
    srslte_softbuffer_rx_reset_tbs(&q->softbuffer, grant->mcs.tbs);
  }
  
  // Uncoment next line to do ZF by default in pdsch_ue example
  //float noise_estimate = 0;
  float noise_estimate = srslte_chest_dl_get_noise_estimate(&q->chest);
  
  if (q->pdsch_cfg.grant.mcs.mod >= 0 && q->pdsch_cfg.grant.mcs.tbs >= 0) {
    ret = srslte_pdsch_decode_rnti(&q->pdsch, &q->pdsch_cfg, &q->softbuffer, 
                                   q->sf_symbols, q->ce, 
                                   noise_estimate, 
                                   rnti, data);
    
    if (ret == SRSLTE_ERROR) {
      q->pkt_errors++;
    } else if (ret == SRSLTE_ERROR_INVALID_INPUTS) {
      fprintf(stderr, "Error calling srslte_pdsch_decode()\n");
    } else if (ret == SRSLTE_SUCCESS) {
      if (SRSLTE_VERBOSE_ISDEBUG()) {
        INFO("Decoded Message: ", 0);
        srslte_vec_fprint_hex(stdout, data, q->pdsch_cfg.grant.mcs.tbs);
      }
    }
    q->pkts_total++;
  }
  return ret; 
}


uint32_t srslte_ue_dl_get_ncce(srslte_ue_dl_t *q) {
  return q->last_location.ncce; 
}

#define MAX_CANDIDATES_UE  16 // From 36.213 Table 9.1.1-1
//#define MAX_CANDIDATES_COM 6 // From 36.213 Table 9.1.1-1
#define MAX_CANDIDATES_COM MAX_CANDIDATES
//#define MAX_CANDIDATES (MAX_CANDIDATES_UE + MAX_CANDIDATES_COM)
typedef struct {
  srslte_dci_format_t format; 
  srslte_dci_location_t loc[MAX_CANDIDATES];
  uint32_t nof_locations; 
} dci_blind_search_t; 

static int dci_blind_search(srslte_ue_dl_t *q, dci_blind_search_t *search_space, uint16_t rnti, srslte_dci_msg_t *dci_msg) 
{
  int ret = SRSLTE_ERROR; 
  uint16_t crc_rem = 0; 
  if (rnti) {
    ret = 0; 
    int i=0;
    while (!ret && i < search_space->nof_locations) {
      INFO("Searching format %s in %d,%d\n",
             srslte_dci_format_string(search_space->format), search_space->loc[i].ncce, search_space->loc[i].L);
      if (srslte_pdcch_decode_msg(&q->pdcch, dci_msg, &search_space->loc[i], search_space->format, &crc_rem)) {
        fprintf(stdout, "Error decoding DCI msg\n");
        return SRSLTE_ERROR;
      } else {
    	  DEBUG("looking for crc %d found msg with format %s in %d,%d with crc %d\n",rnti, srslte_dci_format_string(search_space->format), search_space->loc[i].ncce, search_space->loc[i].L, crc_rem);
      }
      if (crc_rem == rnti) {        
        // If searching for Format1A but found Format0 save it for later 
        if (dci_msg->format == SRSLTE_DCI_FORMAT0 && search_space->format == SRSLTE_DCI_FORMAT1A) 
        {
          q->pending_ul_dci_rnti = crc_rem; 
          memcpy(&q->pending_ul_dci_msg, dci_msg, sizeof(srslte_dci_msg_t));          
        // Else if we found it, save location and leave
        } else if (dci_msg->format == search_space->format) {
          ret = 1; 
          memcpy(&q->last_location, &search_space->loc[i], sizeof(srslte_dci_location_t));          
        } 
      }
      i++; 
    }    
  } else {
    fprintf(stderr, "RNTI not specified\n");
  }
  return ret; 
}

int srslte_ue_dl_find_ul_dci(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  if (rnti) {
    /* Do not search if an UL DCI is already pending */    
    if (q->pending_ul_dci_rnti == rnti) {
      q->pending_ul_dci_rnti = 0;      
      memcpy(dci_msg, &q->pending_ul_dci_msg, sizeof(srslte_dci_msg_t));
      return 1; 
    }
    
    // Configure and run DCI blind search 
    dci_blind_search_t search_space; 
    search_space.format = SRSLTE_DCI_FORMAT0; 
    search_space.nof_locations = srslte_pdcch_ue_locations(&q->pdcch, search_space.loc, MAX_CANDIDATES_UE, sf_idx, cfi, rnti);        
    INFO("Searching UL C-RNTI in %d ue locations\n", search_space.nof_locations);
    return dci_blind_search(q, &search_space, rnti, dci_msg);
  } else {
    return 0; 
  }
}

int srslte_ue_dl_find_dl_dci(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  srslte_rnti_type_t rnti_type; 
  if (rnti == SRSLTE_SIRNTI) {
    rnti_type = SRSLTE_RNTI_SI;
  } else if (rnti == SRSLTE_PRNTI) {
    rnti_type = SRSLTE_RNTI_PCH;    
  } else if (rnti <= SRSLTE_RARNTI_END) {
    rnti_type = SRSLTE_RNTI_RAR;    
  } else {
    rnti_type = SRSLTE_RNTI_USER;
  }
  return srslte_ue_dl_find_dl_dci_type(q, cfi, sf_idx, rnti, rnti_type, dci_msg);
}

// Blind search for SI/P/RA-RNTI
static int find_dl_dci_type_siprarnti(srslte_ue_dl_t *q, uint32_t cfi, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  int ret = 0; 
  // Configure and run DCI blind search 
  dci_blind_search_t search_space; 
  //search_space.nof_locations = srslte_pdcch_ue_locations_all(&q->pdcch, search_space.loc, MAX_CANDIDATES_COM, 0, cfi);
  search_space.nof_locations = srslte_pdcch_common_locations(&q->pdcch, search_space.loc, MAX_CANDIDATES_COM, cfi);
  INFO("Searching SI/P/RA-RNTI in %d common locations, %d formats\n", search_space.nof_locations, nof_common_formats);
  // Search for RNTI only if there is room for the common search space 
  if (search_space.nof_locations > 0) {    
    for (int f=0;f<nof_common_formats;f++) {
      search_space.format = common_formats[f];   
      if ((ret = dci_blind_search(q, &search_space, rnti, dci_msg))) {
        return ret; 
      }
    }
  }
  return SRSLTE_SUCCESS;   
}

// Blind search for C-RNTI
static int find_dl_dci_type_crnti(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  int ret = SRSLTE_SUCCESS; 
  
  // Search UE-specific search space 
  dci_blind_search_t search_space; 
  search_space.nof_locations = srslte_pdcch_ue_locations(&q->pdcch, search_space.loc, MAX_CANDIDATES_UE, sf_idx, cfi, rnti);    
  INFO("Searching DL C-RNTI in %d ue locations, %d formats\n", search_space.nof_locations, nof_ue_formats);
  for (int f=0;f<nof_ue_formats;f++) {
    search_space.format = ue_formats[f];   
    if ((ret = dci_blind_search(q, &search_space, rnti, dci_msg))) {
      return ret; 
    }
  }
  
  // Search Format 1A in the Common SS also
  search_space.format = SRSLTE_DCI_FORMAT1A; 
  search_space.nof_locations = srslte_pdcch_common_locations(&q->pdcch, search_space.loc, MAX_CANDIDATES_COM, cfi);
  // Search for RNTI only if there is room for the common search space 
  if (search_space.nof_locations > 0) {    
    INFO("Searching DL C-RNTI in %d ue locations, format 1A\n", search_space.nof_locations, nof_ue_formats);
    return dci_blind_search(q, &search_space, rnti, dci_msg);   
  }
  return SRSLTE_SUCCESS; 
}

int srslte_ue_dl_find_dci_cc(srslte_ue_dl_t *q, srslte_dci_msg_t *dci_msg, uint32_t cfi, uint32_t sf_idx,
								srslte_rnti_type_t rnti_type, uint32_t sfn)
	/* IMDEA contribution: DCI power analysis */
{
  srslte_dci_location_t locations[MAX_CANDIDATES];
  uint32_t nof_locations;
  uint32_t nof_formats;
  srslte_ra_dl_dci_t dl_dci_unpacked;
  srslte_ra_ul_dci_t ul_dci_unpacked;
  srslte_ra_dl_grant_t dl_grant;
  srslte_ra_ul_grant_t ul_grant;
  srslte_dci_format_t *formats = NULL;
  uint16_t crc_rem = 0;
  bool to_check[MAX_CANDIDATES];
  int lprob[MAX_CANDIDATES];
  float power = 0;

  /* Generate PDCCH candidates */
  nof_locations = srslte_pdcch_ue_locations_all(&q->pdcch, locations, MAX_CANDIDATES, sf_idx, q->cfi);
  formats = ue_all_formats;
  nof_formats = nof_ue_all_formats;

  q->current_rnti = 0xffff;
  q->totRBup = 0;
  q->totRBdw = 0;
  q->totBWup = 0;
  q->totBWdw = 0;

  int ret = 0;
  for (int i=0;i<nof_locations;i++) {
	  to_check[i] = 0;
	  for (int f=0;f<nof_formats;f++) {
		  if (!locations[i].checked) {
			  lprob[i] = (int) round(srslte_pdcch_decode_msg_check(&q->pdcch, dci_msg, &locations[i], formats[f], &crc_rem, q->rnti_list));
//			  printf("t=%d.%d (%d (%d)), ret %d\n", sfn, sf_idx, locations[i].ncce, locations[i].L, ret);
			  if ((formats[f]==SRSLTE_DCI_FORMAT0 && dci_msg->data[0]==1) || (formats[f]==SRSLTE_DCI_FORMAT1A && dci_msg->data[0]==0)) continue;
			  if (lprob[i] > 75) {
				  to_check[i] = 1;
				  if (ret >= PROB_THR) {
					  locations[i].used = 1;
					  //printf("Found location %d with agg %d\n",locations[i].ncce,locations[i].L);
					  for (int j=i;j<nof_locations;j++) {
						  if (locations[j].ncce >= locations[i].ncce && locations[j].ncce < (locations[i].ncce + (1 << locations[i].L))) {
							  locations[j].checked = 1;
							  //printf("skipping location %d with agg %d\n",locations[j].ncce,locations[j].L);
						  }
					  }
				  }
				  if (rnti_in_list(q, crc_rem) || lprob[i] >= PROB_THR) {
					  srslte_ue_dl_reset_rnti_user(q, crc_rem);
					  locations[i].checked = 1;
					  if ((crc_rem >= 1 && crc_rem <= 0xffff)) { // if ((crc_rem >= 0 && crc_rem <= 0x0010)) { //
						//if (lprob[i] < PROB_THR) lprob[i]+=100;
						if (srslte_dci_msg_to_trace(dci_msg, crc_rem, q->cell.nof_prb, q->cell.nof_ports,
								&dl_dci_unpacked, &ul_dci_unpacked, &dl_grant, &ul_grant, sf_idx, sfn, lprob[i],
								locations[i].ncce, locations[i].L, formats[f], q->cfi, power)) {
						  //fprintf(stderr,"1 Error unpacking DCI\n");
						}
						ret++;
						if (crc_rem > 0x0000 && crc_rem <= 0x000a && formats[f]!=SRSLTE_DCI_FORMAT0) q->current_rnti = crc_rem;
						// check upload/download
						if (dl_grant.mcs.tbs>0) {
							q->totRBdw += dl_grant.nof_prb;
							q->totBWdw += dl_grant.mcs.tbs + dl_grant.mcs2.tbs;
							if (q->totRBdw > q->cell.nof_prb) q->totBWdw = q->cell.nof_prb;
						}
						if (ul_grant.mcs.tbs>0) {
							q->totRBup += ul_grant.L_prb;
							q->totBWup += ul_grant.mcs.tbs;
							if (q->totRBup > q->cell.nof_prb) q->totBWup = q->cell.nof_prb;
						}
					  }
					  for (int j=i;j<nof_locations;j++) {
						  if (locations[j].ncce >= locations[i].ncce && locations[j].ncce < (locations[i].ncce + (1 << locations[i].L))) {
							  locations[j].checked = 1;
							  //printf("skipping location %d with agg %d\n",locations[j].ncce,locations[j].L);
						  }
					  }
					  for (int j=0;j<i;j++) {
						  if (locations[i].ncce >= locations[j].ncce && locations[i].ncce < (locations[j].ncce + (1 << locations[j].L))) {
							  locations[j].checked = 1;
						  }
					  }
				  }
			  } else if (lprob[i] == -1) {
				  break;
			  }
		  }
	  }
  }
  for (int i=0;i<nof_locations;i++) {
	  if (to_check[i] && !locations[i].checked && !locations[i].used) fprintf(stderr,"%d\t%d\t%d\t%d\t%d\n",sfn,sf_idx,locations[i].ncce,locations[i].L,q->cfi);
  }
  return ret;
}

float srslte_ue_dl_fix_location_ra(srslte_ue_dl_t *q, srslte_dci_msg_t *dci_msg, uint32_t cfi, uint32_t sf_idx,
								srslte_rnti_type_t rnti_type, uint32_t sfn, uint32_t ncce, uint32_t L, uint8_t print)
	/* IMDEA contribution */
{
  srslte_dci_location_t location;
  uint32_t nof_formats;
  uint32_t best_format = 0;
  srslte_ra_dl_dci_t dl_dci_unpacked;
  srslte_ra_ul_dci_t ul_dci_unpacked;
  srslte_ra_dl_grant_t dl_grant;
  srslte_ra_ul_grant_t ul_grant;
  srslte_dci_format_t *formats = NULL;
  float power = 0;
  float prob;
  uint16_t rnti;
  //float ret = 0, prob_corr = 0;
  float ret = 0;
  //int inlist = 0, rnti_check = 0;;

  q->current_rnti = 0xffff;

  /* Generate PDCCH candidates */
  formats = ue_all_formats;
  nof_formats = nof_ue_all_formats;
  location.ncce = ncce;
  location.L = L;

  for (int f=0;f<nof_formats;f++) {
	prob = srslte_pdcch_decode_msg_check(&q->pdcch, dci_msg, &location, formats[f], &rnti, q->rnti_list);
	if ((formats[f]==SRSLTE_DCI_FORMAT0 && dci_msg->data[0]==1) || (formats[f]==SRSLTE_DCI_FORMAT1A && dci_msg->data[0]==0)) continue;
	//inlist = rnti_in_list_near(q, rnti);
//	if (inlist==1 || prob >= PROB_THR) {
//		prob+=100;
//	} else if (inlist>1) {
//		prob+=50;
//	}
	if (prob > ret) {
		ret = prob;
		best_format = f;
		q->current_rnti = rnti;
//		if (inlist==2) {
//			rnti_check = -1;
//			prob_corr = 50;
//		} else if (inlist==3) {
//			rnti_check = 1;
//			prob_corr = 50;
//		} else if (inlist==1) {
//			rnti_check = 0;
//			prob_corr = 100;
//		} else {
//			rnti_check = 0;
//		}
	}
  }
  if (ret>0) {
  prob = srslte_pdcch_decode_msg_check(&q->pdcch, dci_msg, &location, formats[best_format], &rnti, q->rnti_list);
  	  if (print) {
		  //srslte_dci_msg_to_trace(dci_msg, rnti+rnti_check, q->cell.nof_prb, &dl_dci_unpacked, &ul_dci_unpacked, &dl_grant, &ul_grant, sf_idx, sfn, prob+corr, ncce, L, q->cfi, power);
		  srslte_dci_msg_to_trace(dci_msg, rnti, q->cell.nof_prb, q->cell.nof_ports, &dl_dci_unpacked, &ul_dci_unpacked, &dl_grant, &ul_grant, sf_idx, sfn, prob, ncce, L, formats[best_format], q->cfi, power);
	  }
  }
  return ret;
}

int srslte_ue_dl_find_dci_power(srslte_ue_dl_t *q, srslte_dci_msg_t *dci_msg, uint32_t cfi, uint32_t sf_idx,
								srslte_rnti_type_t rnti_type, uint32_t sfn)
	/* IMDEA contribution: DCI power analysis */
{
  srslte_dci_location_t locations[MAX_CANDIDATES];
  uint32_t nof_locations;


  /* Generate PDCCH candidates */
  nof_locations = srslte_pdcch_ue_locations_all(&q->pdcch, locations, MAX_CANDIDATES, sf_idx, q->cfi);

  int ret = 0;
  for (int i=0;i<nof_locations;i++) {
      q->last_n_cce = locations[i].ncce;
      if (!locations[i].checked) {
    	  ret = srslte_pdcch_decode_msg_power(&q->pdcch, dci_msg, &locations[i], sfn, sf_idx);
    	  if (ret >= 1) {
    		  locations[i].used = 1;
    		  //printf("Found location %d with agg %d\n",locations[i].ncce,locations[i].L);
    		  for (int j=i;j<nof_locations;j++) {
    			  if (locations[j].ncce >= locations[i].ncce && locations[j].ncce < (locations[i].ncce + (1 << locations[i].L))) {
    				  locations[j].checked = 1;
    				  //printf("skipping location %d with agg %d\n",locations[j].ncce,locations[j].L);
    			  }
    		  }
    	  }
      }
  }
  return ret;
}

int srslte_ue_dl_get_control_cc(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t sf_idx, uint32_t rvidx, uint32_t sfn)
{
  srslte_dci_msg_t dci_msg;
  int ret = SRSLTE_ERROR;

  if ((ret = srslte_ue_dl_decode_fft_estimate(q, input, sf_idx, &q->cfi)) < 0) {
    return ret;
  }

  if (srslte_pdcch_extract_llr(&q->pdcch, q->sf_symbols, q->ce, 0, sf_idx, q->cfi)) {
    fprintf(stderr, "Error extracting LLRs\n");
    return SRSLTE_ERROR;
  }

#ifndef POWER_ONLY
  return srslte_ue_dl_find_dci_cc(q, &dci_msg, q->cfi, sf_idx, SRSLTE_RNTI_USER, sfn);
#else
  return srslte_ue_dl_find_dci_power(q, &dci_msg, q->cfi, sf_idx, SRSLTE_RNTI_USER, sfn);
#endif
}

int srslte_ue_dl_find_dl_dci_type(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, 
                                  uint16_t rnti, srslte_rnti_type_t rnti_type, srslte_dci_msg_t *dci_msg)
{  
  if (rnti_type == SRSLTE_RNTI_SI || rnti_type == SRSLTE_RNTI_PCH || rnti_type == SRSLTE_RNTI_RAR) {
    return find_dl_dci_type_siprarnti(q, cfi, rnti, dci_msg);
  } else {
    return find_dl_dci_type_crnti(q, cfi, sf_idx, rnti, dci_msg);
  }
}

float srslte_ue_dl_fix_control_ra(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t sf_idx, uint32_t rvidx, uint32_t sfn, uint32_t ncce, uint32_t L, uint32_t cfi, uint8_t print)
{
  srslte_dci_msg_t dci_msg;
  int ret = SRSLTE_ERROR;

  if ((ret = srslte_ue_dl_decode_fft_estimate(q, input, sf_idx, &q->cfi)) < 0) {
      return ret;
  }
  if (q->cfi != cfi) return SRSLTE_ERROR;

  if (srslte_pdcch_extract_llr(&q->pdcch, q->sf_symbols, q->ce, 0, sf_idx, q->cfi)) {
    fprintf(stderr, "Error extracting LLRs\n");
    return SRSLTE_ERROR;
  }

  return srslte_ue_dl_fix_location_ra(q, &dci_msg, q->cfi, sf_idx, SRSLTE_RNTI_USER, sfn, ncce, L, print);
}

int srslte_ue_dl_decode_broad(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t sf_idx, uint16_t rnti)
{
  srslte_dci_msg_t dci_msg;
  srslte_ra_dl_dci_t dci_unpacked;
  srslte_ra_dl_grant_t grant;
  int ret = SRSLTE_ERROR;

  if ((ret = srslte_ue_dl_decode_fft_estimate(q, input, sf_idx, &q->cfi)) < 0) {
    return ret;
  }

  if (srslte_pdcch_extract_llr(&q->pdcch, q->sf_symbols, q->ce, 0, sf_idx, q->cfi)) {
    fprintf(stderr, "Error extracting LLRs\n");
    return SRSLTE_ERROR;
  }

  int found_dci = srslte_ue_dl_find_dl_dci(q, q->cfi, sf_idx, rnti, &dci_msg);

  if (found_dci == 1) {
	if (srslte_dci_msg_to_dl_grant(&dci_msg, rnti, q->cell.nof_prb, q->cell.nof_ports, &dci_unpacked, &grant)) {
      fprintf(stderr, "Error unpacking DCI\n");
      return SRSLTE_ERROR;
    }

    ret = srslte_ue_dl_decode_rnti_rv_packet(q, &grant, data, q->cfi, sf_idx, rnti, q->dl_dci.rv_idx);
  }


  if (found_dci == 1 && ret == SRSLTE_SUCCESS) {
    return q->pdsch_cfg.grant.mcs.tbs;
  } else {
    return 0;
  }
}

int srslte_ue_dl_decode_rnti_rv(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t sf_idx, uint16_t rnti, uint32_t rvidx) 
{
  srslte_dci_msg_t dci_msg;
  srslte_ra_dl_dci_t dci_unpacked;
  srslte_ra_dl_grant_t grant; 
  int ret = SRSLTE_ERROR; 
  uint32_t cfi;
  
  
  if ((ret = srslte_ue_dl_decode_fft_estimate(q, input, sf_idx, &cfi)) < 0) {
    return ret; 
  }
  
  if (srslte_pdcch_extract_llr(&q->pdcch, q->sf_symbols, q->ce, srslte_chest_dl_get_noise_estimate(&q->chest), sf_idx, cfi)) {
    fprintf(stderr, "Error extracting LLRs\n");
    return SRSLTE_ERROR;
  }

  int found_dci = srslte_ue_dl_find_dl_dci(q, cfi, sf_idx, rnti, &dci_msg);   
  if (found_dci == 1) {
    
    if (srslte_dci_msg_to_dl_grant(&dci_msg, rnti, q->cell.nof_prb, q->cell.nof_ports, &dci_unpacked, &grant)) {
      fprintf(stderr, "Error unpacking DCI\n");
      return SRSLTE_ERROR;   
    }

    ret = srslte_ue_dl_decode_rnti_rv_packet(q, &grant, data, cfi, sf_idx, rnti, rvidx);    
  }
   
  if (found_dci == 1 && ret == SRSLTE_SUCCESS) { 
    return q->pdsch_cfg.grant.mcs.tbs;    
  } else {
    return 0;
  }
}


bool srslte_ue_dl_decode_phich(srslte_ue_dl_t *q, uint32_t sf_idx, uint32_t n_prb_lowest, uint32_t n_dmrs)
{
  uint8_t ack_bit; 
  float distance;
  uint32_t ngroup, nseq; 
  srslte_phich_calc(&q->phich, n_prb_lowest, n_dmrs, &ngroup, &nseq);
  DEBUG("Decoding PHICH sf_idx=%d, n_prb_lowest=%d, n_dmrs=%d, n_group=%d, n_seq=%d\n", 
    sf_idx, n_prb_lowest, n_dmrs, ngroup, nseq);
  if (!srslte_phich_decode(&q->phich, q->sf_symbols, q->ce, 0, ngroup, nseq, sf_idx, &ack_bit, &distance)) {
    INFO("Decoded PHICH %d with distance %f\n", ack_bit, distance);    
  } else {
    fprintf(stderr, "Error decoding PHICH\n");
    return false; 
  }
  if (ack_bit) {
    return true; 
  } else {
    return false; 
  }
}

void srslte_ue_dl_save_signal(srslte_ue_dl_t *q, srslte_softbuffer_rx_t *softbuffer, uint32_t tti, uint32_t rv_idx, uint16_t rnti, uint32_t cfi) {
  srslte_vec_save_file("sf_symbols", q->sf_symbols, SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)*sizeof(cf_t));
  printf("%d samples\n", SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp));
  srslte_vec_save_file("ce0", q->ce[0], SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)*sizeof(cf_t));
  if (q->cell.nof_ports > 1) {
    srslte_vec_save_file("ce1", q->ce[1], SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)*sizeof(cf_t));
  }
  srslte_vec_save_file("pcfich_ce0", q->pcfich.ce[0], q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_ce1", q->pcfich.ce[1], q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_symbols", q->pcfich.symbols[0], q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_eq_symbols", q->pcfich.d, q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_llr", q->pcfich.data_f, PCFICH_CFI_LEN*sizeof(float));
  
  srslte_vec_save_file("pdcch_ce0", q->pdcch.ce[0], q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_ce1", q->pdcch.ce[1], q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_symbols", q->pdcch.symbols[0], q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_eq_symbols", q->pdcch.d, q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_llr", q->pdcch.llr, q->pdcch.nof_cce*72*sizeof(float));
  
  
  srslte_vec_save_file("pdsch_symbols", q->pdsch.d, q->pdsch_cfg.nbits.nof_re*sizeof(cf_t));
  srslte_vec_save_file("llr", q->pdsch.e, q->pdsch_cfg.nbits.nof_bits*sizeof(cf_t));
  int cb_len = q->pdsch_cfg.cb_segm.K1; 
  for (int i=0;i<q->pdsch_cfg.cb_segm.C;i++) {
    char tmpstr[64]; 
    snprintf(tmpstr,64,"rmout_%d.dat",i);
    srslte_vec_save_file(tmpstr, softbuffer->buffer_f[i], (3*cb_len+12)*sizeof(int16_t));  
  }
  printf("Saved files for tti=%d, sf=%d, cfi=%d, mcs=%d, rv=%d, rnti=%d\n", tti, tti%10, cfi, 
         q->pdsch_cfg.grant.mcs.idx, rv_idx, rnti);
}



