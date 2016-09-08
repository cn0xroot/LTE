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

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <stdbool.h>

#include "srslte/srslte.h"
#include "srslte/rf/rf.h"
#include "srslte/rf/rf_utils.h"

#define ENABLE_AGC_DEFAULT
//#define CORRECT_SAMPLE_OFFSET

char *output_file_name = NULL;
char *rf_args="";
float rf_gain=60.0, rf_freq=-1.0;
int nof_prb = 6;
int nof_subframes = -1;
int N_id_2 = -1;
bool disable_cfo = false;

cell_search_cfg_t cell_detect_config = {
  SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
  SRSLTE_DEFAULT_MAX_FRAMES_PSS,
  SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
  0
};

enum receiver_state { DECODE_MIB 	} state;

bool go_exit = false;
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

void usage(char *prog) {
  printf("Usage: %s [aCgpnv] -l N_id_2 -f rx_frequency_hz -o output_file\n", prog);
  printf("\t-a RF args [Default %s]\n", rf_args);
  printf("\t-C disable cfo correction [Default %s]\n", rf_args);
  printf("\t-g RF Gain [Default %.2f dB]\n", rf_gain);
  printf("\t-p nof_prb [Default %d]\n", nof_prb);
  printf("\t-n nof_subframes [Default %d]\n", nof_subframes);
  printf("\t-v verbose\n");
}

void parse_args(int argc, char **argv) {
  int opt;
  while ((opt = getopt(argc, argv, "agpnvfol")) != -1) {
    switch (opt) {
    case 'o':
      output_file_name = argv[optind];
      break;
    case 'a':
      rf_args = argv[optind];
      break;
    case 'C':
      disable_cfo = true;
	  break;
	case 'g':
      rf_gain = atof(argv[optind]);
      break;
    case 'p':
      nof_prb = atoi(argv[optind]);
      break;
    case 'f':
      rf_freq = atof(argv[optind]);
      break;
    case 'n':
      nof_subframes = atoi(argv[optind]);
      break;
    case 'l':
      N_id_2 = atoi(argv[optind]);
      break;
    case 'v':
      srslte_verbose++;
      break;
    default:
      usage(argv[0]);
      exit(-1);
    }
  }
  if (&rf_freq < 0 || N_id_2 == -1 || output_file_name == NULL) {
    usage(argv[0]);
    exit(-1);
  }
}

int srslte_rf_recv_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  return srslte_rf_recv(h, data, nsamples, 1);
}

double srslte_rf_set_rx_gain_th_wrapper(void *h, double f) {
  return srslte_rf_set_rx_gain_th((srslte_rf_t*) h, f);
}

int main(int argc, char **argv) {
  int n, ret;
  srslte_rf_t rf;
  srslte_filesink_t sink;
  srslte_ue_sync_t ue_sync; 
  srslte_cell_t cell; 
  int64_t sf_cnt, sf_guard;
  srslte_ue_mib_t ue_mib;
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  int sfn_offset;
  bool fstart = 0;
  float cfo = 0;

  srslte_ue_dl_t ue_dl;

  uint32_t sfn = 0; // system frame number
  cf_t *sf_buffer = NULL;

  parse_args(argc, argv);
  
  srslte_filesink_init(&sink, output_file_name, SRSLTE_COMPLEX_FLOAT_BIN);

  printf("Opening RF device...\n");
  if (srslte_rf_open(&rf, rf_args)) {
    fprintf(stderr, "Error opening rf\n");
    exit(-1);
  }
  srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

  // The new version disables the AGC by default (let's see if it is good or not)
  /* Set receiver gain */
  if (rf_gain > 0) {
	srslte_rf_set_rx_gain(&rf, rf_gain);
  } else {
	printf("Starting AGC thread...\n");
	if (srslte_rf_start_gain_thread(&rf, false)) {
	  fprintf(stderr, "Error opening rf\n");
	  exit(-1);
	}
	srslte_rf_set_rx_gain(&rf, 50);
	cell_detect_config.init_agc = 50;
  }

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigprocmask(SIG_UNBLOCK, &sigset, NULL);
    signal(SIGINT, sig_int_handler);

  /* set receiver frequency and gain*/
  printf("Set RX freq: %.6f MHz\n", srslte_rf_set_rx_freq(&rf, rf_freq) / 1000000);
  printf("Set RX gain: %.1f dB\n", srslte_rf_set_rx_gain(&rf, rf_gain));
    int srate = srslte_sampling_freq_hz(nof_prb);    
    if (srate != -1) {  
      if (srate < 10e6) {          
        srslte_rf_set_master_clock_rate(&rf, 4*srate);        
      } else {
        srslte_rf_set_master_clock_rate(&rf, srate);        
      }
      printf("Setting sampling rate %.2f MHz\n", (float) srate/1000000);
      float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
      if (srate_rf != srate) {
        fprintf(stderr, "Could not set sampling rate\n");
        exit(-1);
      }
    } else {
      fprintf(stderr, "Invalid number of PRB %d\n", nof_prb);
      exit(-1);
    }
  srslte_rf_rx_wait_lo_locked(&rf);
  srslte_rf_start_rx_stream(&rf);

  uint32_t ntrial=0;
  uint32_t max_trial=3;
  do {
	  ret = rf_search_and_decode_mib(&rf, &cell_detect_config, N_id_2, &cell, &cfo);
	  if (ret < 0) {
		  fprintf(stderr, "Error searching for cell\n");
		  exit(-1);
	  } else if (ret == 0 && !go_exit) {
		  printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
	  }
	  if (ntrial >= max_trial) go_exit = true;
  } while (ret == 0 && !go_exit);

  if (go_exit) {
	  exit(-1);
  }

  /* set sampling frequency */
  srate = srslte_sampling_freq_hz(cell.nof_prb);
  if (srate != -1) {
	  if (srate < 10e6) {
		  srslte_rf_set_master_clock_rate(&rf, 4*srate);
	  } else {
		  srslte_rf_set_master_clock_rate(&rf, srate);
	  }
	  printf("Setting sampling rate %.2f MHz\n", (float) srate/1000000);
	  float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
	  if (srate_rf != srate) {
		  fprintf(stderr, "Could not set sampling rate\n");
		  exit(-1);
	  }
  } else {
	  fprintf(stderr, "Invalid number of PRB %d\n", cell.nof_prb);
	  exit(-1);
  }

  INFO("Stopping RF and flushing buffer...\r",0);
  srslte_rf_stop_rx_stream(&rf);
  srslte_rf_flush_buffer(&rf);
  
  /* If reading from file, go straight to PDSCH decoding. Otherwise, decode MIB first */
  if (srslte_ue_sync_init(&ue_sync, cell, srslte_rf_recv_wrapper, (void*) &rf)) {
	  fprintf(stderr, "Error initiating ue_sync\n");
	  exit(-1);
  }

  state = DECODE_MIB;

  if (srslte_ue_mib_init(&ue_mib, cell)) {
    fprintf(stderr, "Error initaiting UE MIB decoder\n");
    exit(-1);
  }

  if (srslte_ue_dl_init(&ue_dl, cell)) {  // This is the User RNTI
    fprintf(stderr, "Error initiating UE downlink processing module\n");
    exit(-1);
  }

  /* Initialize subframe counter */
  sf_cnt = 0;
  sf_guard = 0;

  srslte_rf_start_rx_stream(&rf);

  if (rf_gain < 0) {
    srslte_ue_sync_start_agc(&ue_sync, srslte_rf_set_rx_gain_th_wrapper, cell_detect_config.init_agc);
  }
#ifdef PRINT_CHANGE_SCHEDULIGN
  srslte_ra_dl_dci_t old_dl_dci;
  bzero(&old_dl_dci, sizeof(srslte_ra_dl_dci_t));
#endif

  ue_sync.correct_cfo = !disable_cfo;

  INFO("\nEntering main loop...\n\n", 0);
  /* Main loop */
  while (!go_exit && (sf_cnt < nof_subframes || nof_subframes == -1)) {

    ret = srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
    if (ret < 0) {
      fprintf(stderr, "Error calling srslte_ue_sync_work()\n");
    }

    /* srslte_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
    if (ret == 1) {
      switch (state) {
        case DECODE_MIB:
          if (srslte_ue_sync_get_sfidx(&ue_sync) == 0) {
            srslte_pbch_decode_reset(&ue_mib.pbch);
            n = srslte_ue_mib_decode(&ue_mib, sf_buffer, bch_payload, NULL, &sfn_offset);
            if (n < 0) {
              fprintf(stderr, "Error decoding UE MIB\n");
              exit(-1);
            } else if (n == SRSLTE_UE_MIB_FOUND) {
              srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
              //srslte_cell_fprint(stdout, &cell, sfn);
              fprintf(stdout,"Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
              //exit(0);
              if (!fstart) {
            	  fprintf(stderr,"*************************\n*************************\nRecording started: %d\n*************************\n*************************\n", sfn, sfn_offset);
            	  sfn = (sfn + sfn_offset)%1024;
            	  fstart = 1;
              }
            } else {
              fprintf(stdout,"MIB not decoded. SFN: %d, offset: %d\n", sfn, sfn_offset);
            }
          }
          break;
      }
      if (srslte_ue_sync_get_sfidx(&ue_sync) == 9) {
        sfn++;
//        if (sfn == 1024) {
//          sfn = 0;
//        }
      }
    if (fstart) srslte_filesink_write(&sink, sf_buffer, SRSLTE_SF_LEN_PRB(cell.nof_prb));
    } else if (ret == 0) {
    	if (fstart) {
    		fprintf(stderr,"Sync loss at %d\n", sfn);
    		go_exit = true;
    	} else {
    		fprintf(stderr,"Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\n",
    				srslte_sync_get_peak_value(&ue_sync.sfind),
					ue_sync.frame_total_cnt, ue_sync.state);
    	}

    }

    if (fstart) sf_cnt++;
    sf_guard++;
//    if (sf_guard > nof_subframes + 10000) {
//      fprintf(stderr,"watchdog exit\n");
//	  go_exit = true;
//    }
  } // Main loop

  srslte_ue_dl_free(&ue_dl);
  srslte_ue_sync_free(&ue_sync);
  
  srslte_ue_mib_free(&ue_mib);
  srslte_rf_close(&rf);
//  printf("\nBye\n");
  if (go_exit) {
	  exit(-1);
  } else {
	  exit(0);
  }
}

