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
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>

#include "srslte/srslte.h"

#define ENABLE_AGC_DEFAULT
//#define CORRECT_SAMPLE_OFFSET

#ifndef DISABLE_RF
#include "srslte/rf/rf.h"
#include "srslte/rf/rf_utils.h"

cell_search_cfg_t cell_detect_config = {
  SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
  SRSLTE_DEFAULT_MAX_FRAMES_PSS,
  SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
  0
};

#else
#warning Compiling pdsch_ue with no RF support
#endif

/**********************************************************************
 *  Program arguments processing
 ***********************************************************************/
typedef struct {
  int nof_subframes;
  bool disable_cfo; 
  uint32_t time_offset; 
  int force_N_id_2;
  uint16_t rnti;
  char *input_file_name;
  char *rnti_list_file;
  char *rnti_list_file_out;
  int file_offset_time; 
  float file_offset_freq;
  uint32_t file_nof_prb;
  uint32_t file_nof_ports;
  uint32_t file_cell_id;
  char *rf_args; 
  double rf_freq; 
  float rf_gain;
  int net_port; 
  char *net_address; 
  int net_port_signal; 
  char *net_address_signal;   
}prog_args_t;

void args_default(prog_args_t *args) {
  args->nof_subframes = -1;
  args->rnti = SRSLTE_SIRNTI;
  args->force_N_id_2 = -1; // Pick the best
  args->input_file_name = NULL;
  args->rnti_list_file = NULL;
  args->rnti_list_file_out = NULL;
  args->disable_cfo = false; 
  args->time_offset = 0; 
  args->file_nof_prb = 25; 
  args->file_nof_ports = 1; 
  args->file_cell_id = 0; 
  args->file_offset_time = 0; 
  args->file_offset_freq = 0; 
  args->rf_args = "";
  args->rf_freq = -1.0;
#ifdef ENABLE_AGC_DEFAULT
  args->rf_gain = -1.0; 
#else
  args->rf_gain = 50.0;
#endif
  args->net_port = -1; 
  args->net_address = "127.0.0.1";
  args->net_port_signal = -1; 
  args->net_address_signal = "127.0.0.1";
}

void usage(prog_args_t *args, char *prog) {
  printf("Usage: %s [agpPoOcilntuv] -f rx_frequency (in Hz) | -i input_file\n", prog);
#ifndef DISABLE_RF
  printf("\t-a RF args [Default %s]\n", args->rf_args);
#ifdef ENABLE_AGC_DEFAULT
  printf("\t-g RF fix RX gain [Default AGC]\n");
#else
  printf("\t-g Set RX gain [Default %.1f dB]\n", args->rf_gain);
#endif
#else
  printf("\t   RF is disabled.\n");
#endif
  printf("\t-i input_file [Default use RF board]\n");
  printf("\t-o offset frequency correction (in Hz) for input file [Default %.1f Hz]\n", args->file_offset_freq);
  printf("\t-O offset samples for input file [Default %d]\n", args->file_offset_time);
  printf("\t-p nof_prb for input file [Default %d]\n", args->file_nof_prb);
  printf("\t-P nof_ports for input file [Default %d]\n", args->file_nof_ports);
  printf("\t-c cell_id for input file [Default %d]\n", args->file_cell_id);
  printf("\t-l Force N_id_2 [Default best]\n");
  printf("\t-C Disable CFO correction [Default %s]\n", args->disable_cfo?"Disabled":"Enabled");
  printf("\t-t Add time offset [Default %d]\n", args->time_offset);
  printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
  printf("\t-s remote UDP port to send input signal (-1 does nothing with it) [Default %d]\n", args->net_port_signal);
  printf("\t-S remote UDP address to send input signal [Default %s]\n", args->net_address_signal);
  printf("\t-u remote TCP port to send data (-1 does nothing with it) [Default %d]\n", args->net_port);
  printf("\t-U remote TCP address to send data [Default %s]\n", args->net_address);
  printf("\t-v [set srslte_verbose to debug, default none]\n");
  printf("\t-z filename of the output reporting one int per rnti (tot length 64k entries)\n");
  printf("\t-Z filename of the input reporting one int per rnti (tot length 64k entries)\n");
}

void parse_args(prog_args_t *args, int argc, char **argv) {
  int opt;
  args_default(args);
  while ((opt = getopt(argc, argv, "aoglipPcOCtnvfuUsSzZ")) != -1) {
    switch (opt) {
    case 'i':
      args->input_file_name = argv[optind];
      break;
    case 'p':
      args->file_nof_prb = atoi(argv[optind]);
      break;
    case 'P':
      args->file_nof_ports = atoi(argv[optind]);
      break;
    case 'o':
      args->file_offset_freq = atof(argv[optind]);
      break;
    case 'O':
      args->file_offset_time = atoi(argv[optind]);
      break;
    case 'c':
      args->file_cell_id = atoi(argv[optind]);
      break;
    case 'a':
      args->rf_args = argv[optind];
      break;
    case 'g':
      args->rf_gain = atof(argv[optind]);
      break;
    case 'C':
      args->disable_cfo = true;
      break;
    case 't':
      args->time_offset = atoi(argv[optind]);
      break;
    case 'f':
      args->rf_freq = strtod(argv[optind], NULL);
      break;
    case 'n':
      args->nof_subframes = atoi(argv[optind]);
      break;
    case 'l':
      args->force_N_id_2 = atoi(argv[optind]);
      break;
    case 'u':
      args->net_port = atoi(argv[optind]);
      break;
    case 'U':
      args->net_address = argv[optind];
      break;
    case 's':
      args->net_port_signal = atoi(argv[optind]);
      break;
    case 'S':
      args->net_address_signal = argv[optind];
      break;
    case 'v':
      srslte_verbose++;
      break;
    case 'z':
   	  args->rnti_list_file_out = argv[optind];
   	  break;
    case 'Z':
   	  args->rnti_list_file = argv[optind];
   	  break;
    default:
      usage(args, argv[0]);
      exit(-1);
    }
  }
  if (args->rf_freq < 0 && args->input_file_name == NULL) {
    usage(args, argv[0]);
    exit(-1);
  }
}
/**********************************************************************/

/* TODO: Do something with the output data */
uint8_t data[20000];

bool go_exit = false; 
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

#ifndef DISABLE_RF
int srslte_rf_recv_wrapper(void *h, void *data, uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  return srslte_rf_recv(h, data, nsamples, 1);
}

double srslte_rf_set_rx_gain_th_wrapper(void *h, double f) {
  return srslte_rf_set_rx_gain_th((srslte_rf_t*) h, f);
}

#endif

extern float mean_exec_time;

enum receiver_state { DECODE_MIB, DECODE_PDSCH} state;

srslte_ue_dl_t ue_dl; 
srslte_ue_sync_t ue_sync; 
prog_args_t prog_args; 

uint32_t sfn = 0; // system frame number
cf_t *sf_buffer = NULL; 
srslte_netsink_t net_sink, net_sink_signal; 

int main(int argc, char **argv) {
  int ret; 
  srslte_cell_t cell;  
  int64_t sf_cnt;
  srslte_ue_mib_t ue_mib; 
#ifndef DISABLE_RF
  srslte_rf_t rf; 
#endif
  int n; 
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  int sfn_offset;
  float cfo = 0;
  FILE *fid;
  uint16_t rnti_tmp;

  parse_args(&prog_args, argc, argv);

  if (prog_args.net_port > 0) {
    if (srslte_netsink_init(&net_sink, prog_args.net_address, prog_args.net_port, SRSLTE_NETSINK_TCP)) {
      fprintf(stderr, "Error initiating UDP socket to %s:%d\n", prog_args.net_address, prog_args.net_port);
      exit(-1);
    }
    srslte_netsink_set_nonblocking(&net_sink);
  }
  if (prog_args.net_port_signal > 0) {
    if (srslte_netsink_init(&net_sink_signal, prog_args.net_address_signal, 
      prog_args.net_port_signal, SRSLTE_NETSINK_UDP)) {
      fprintf(stderr, "Error initiating UDP socket to %s:%d\n", prog_args.net_address_signal, prog_args.net_port_signal);
      exit(-1);
    }
    srslte_netsink_set_nonblocking(&net_sink_signal);
  }
  
#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    
    printf("Opening RF device...\n");
    if (srslte_rf_open(&rf, prog_args.rf_args)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    /* Set receiver gain */
    if (prog_args.rf_gain > 0) {
      srslte_rf_set_rx_gain(&rf, prog_args.rf_gain);      
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
    
    srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

    /* set receiver frequency */
    printf("Tuning receiver to %.3f MHz\n", prog_args.rf_freq/1000000);
    srslte_rf_set_rx_freq(&rf, prog_args.rf_freq);
    srslte_rf_rx_wait_lo_locked(&rf);

    uint32_t ntrial=0; 
    do {
      ret = rf_search_and_decode_mib(&rf, &cell_detect_config, prog_args.force_N_id_2, &cell, &cfo);
      if (ret < 0) {
        fprintf(stderr, "Error searching for cell\n");
        exit(-1); 
      } else if (ret == 0 && !go_exit) {
        printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
      }      
    } while (ret == 0 && !go_exit); 
    
    if (go_exit) {
      exit(0);
    }
    /* set sampling frequency */
    int srate = srslte_sampling_freq_hz(cell.nof_prb);    
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
  }
#endif
  
  /* If reading from file, go straight to PDSCH decoding. Otherwise, decode MIB first */
  if (prog_args.input_file_name) {
    /* preset cell configuration */
    cell.id = prog_args.file_cell_id; 
    cell.cp = SRSLTE_CP_NORM; 
    cell.phich_length = SRSLTE_PHICH_NORM;
    cell.phich_resources = SRSLTE_PHICH_R_1;
    cell.nof_ports = prog_args.file_nof_ports; 
    cell.nof_prb = prog_args.file_nof_prb; 
    
    if (srslte_ue_sync_init_file(&ue_sync, prog_args.file_nof_prb, 
      prog_args.input_file_name, prog_args.file_offset_time, prog_args.file_offset_freq)) {
      fprintf(stderr, "Error initiating ue_sync\n");
      exit(-1); 
    }

  } else {
#ifndef DISABLE_RF
    if (srslte_ue_sync_init(&ue_sync, cell, srslte_rf_recv_wrapper, (void*) &rf)) {
      fprintf(stderr, "Error initiating ue_sync\n");
      exit(-1); 
    }
#endif
  }

  state = DECODE_MIB; 

  if (srslte_ue_mib_init(&ue_mib, cell)) {
    fprintf(stderr, "Error initiating UE MIB decoder\n");
    exit(-1);
  }    

  if (srslte_ue_dl_init(&ue_dl, cell)) {  // This is the User RNTI
    fprintf(stderr, "Error initiating UE downlink processing module\n");
    exit(-1);
  }
  
  /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
  srslte_ue_dl_set_rnti(&ue_dl, prog_args.rnti);

  /* Initialize subframe counter */
  sf_cnt = 0;

#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_rf_start_rx_stream(&rf);    
  }
#endif
    
#ifndef DISABLE_RF
  if (prog_args.rf_gain < 0) {
    srslte_ue_sync_start_agc(&ue_sync, srslte_rf_set_rx_gain_th_wrapper, cell_detect_config.init_agc);    
  }
#endif
#ifdef PRINT_CHANGE_SCHEDULIGN
  srslte_ra_dl_dci_t old_dl_dci; 
  bzero(&old_dl_dci, sizeof(srslte_ra_dl_dci_t));
#endif
  
  ue_sync.correct_cfo = !prog_args.disable_cfo;

  if (prog_args.rnti_list_file != NULL) {
	  fid = fopen(prog_args.rnti_list_file,"r");
	  if (fid>0) {
		  for (int i=0;i<65536;i++) {
			if (fscanf(fid,"%d",&rnti_tmp) != EOF) {
	//		  if (rnti_tmp) printf("rnti %d val %d\n", i, rnti_tmp);
				srslte_ue_dl_reset_rnti_user_to(&ue_dl, i, rnti_tmp); // check this
			  //printf("is rnti in the list? %d\n",rnti_in_list(&ue_dl, rnti_tmp));
			}
		  }
		  fclose(fid);
	  }
  }
  // Set initial CFO for ue_sync
  srslte_ue_sync_set_cfo(&ue_sync, cfo);

  INFO("\nEntering main loop...\n\n", 0);
  /* Main loop */
  while (!go_exit && (sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1)) {
    
    ret = srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
    if (ret < 0) {
      fprintf(stderr, "Error calling srslte_ue_sync_work()\n");
    }
    if (ret == 7) {
    	go_exit = true;
    }

#ifdef CORRECT_SAMPLE_OFFSET
    float sample_offset = (float) srslte_ue_sync_get_last_sample_offset(&ue_sync)+srslte_ue_sync_get_sfo(&ue_sync)/1000;
    srslte_ue_dl_set_sample_offset(&ue_dl, sample_offset);
#endif

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
              //printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
              sfn = (sfn + sfn_offset)%1024; 
              state = DECODE_PDSCH;
            }
          }
        case DECODE_PDSCH:
        	srslte_ue_dl_get_control_cc(&ue_dl, &sf_buffer[prog_args.time_offset], data, srslte_ue_sync_get_sfidx(&ue_sync), 0, sfn);
#ifndef POWER_ONLY
        	if (ue_dl.current_rnti != 0xffff) {
        		n = srslte_ue_dl_decode_broad(&ue_dl, &sf_buffer[prog_args.time_offset], data, srslte_ue_sync_get_sfidx(&ue_sync), ue_dl.current_rnti-1);
				if (n > 0) {
					srslte_ue_dl_reset_rnti_user(&ue_dl, 256*data[(n/8)-2] + data[(n/8)-1]);
// 					ue_dl.rnti_list[256*data[(n/8)-2] + data[(n/8)-1]] = 1;
//        			printf("%d\t%d\t%d\t", sfn, srslte_ue_sync_get_sfidx(&ue_sync), 256*data[(n/8)-2] + data[(n/8)-1]);
//        			for (int k=0; k<(n/8); k++) {
//        				printf("%02x ",data[k]);
//        			}
//        			printf("\n");
				}
        	}
#endif
        	break;
      }
      if (srslte_ue_sync_get_sfidx(&ue_sync) == 9) {
        sfn++; 
        if (sfn % 1024 == 0) {
        	srslte_ue_dl_update_rnti_list(&ue_dl);
        }
      }
      
    } else if (ret == 0) {
      printf("Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\r", 
        srslte_sync_get_peak_value(&ue_sync.sfind), 
        ue_sync.frame_total_cnt, ue_sync.state);      
    }
    sf_cnt++;                  
  } // Main loop
  
  if (prog_args.rnti_list_file_out != NULL) {
	  fid = fopen(prog_args.rnti_list_file_out,"w");
	  for (int i=0;i<65536;i++) {
		if (i<10) {
			fprintf(fid,"%d\n",2);
		} else {
//			if (rnti_in_list(&ue_dl, i)) printf("%d val %d (%d)\n",i,ue_dl.rnti_list[i], ue_dl.rnti_cnt[i]);
		  	if (ue_dl.rnti_cnt[i] >= 10) fprintf(fid,"%d\n",ue_dl.rnti_list[i]);
		  	else fprintf(fid,"%d\n",0);
		}
	  }
	  fclose(fid);
  }

  srslte_ue_dl_free(&ue_dl);
  srslte_ue_sync_free(&ue_sync);
  
#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_ue_mib_free(&ue_mib);
    srslte_rf_close(&rf);    
  }
#endif
  exit(0);
}
