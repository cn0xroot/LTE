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


#ifndef DISABLE_RF
#include "srslte/rf/rf.h"
#include "srslte/rf/rf_utils.h"

cell_search_cfg_t cell_detect_config = {
  5000,
  200, // nof_frames_total
  10.0 // threshold
};
#else
#warning Compiling pdsch_ue with no RF support
#endif

#define PRINT_CHANGE_SCHEDULIGN

/**********************************************************************
 *  Program arguments processing
 ***********************************************************************/
typedef struct {
  int nof_subframes;
  bool disable_plots;
  bool disable_plots_except_constellation;
  bool disable_cfo; 
  uint32_t time_offset; 
  int force_N_id_2;
  uint16_t rnti;
  char *input_file_name;
  char *to_fix;
  char *rnti_to_fix;
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
  args->disable_plots = false;
  args->disable_plots_except_constellation = false;
  args->nof_subframes = -1;
  args->rnti = SRSLTE_SIRNTI;
  args->force_N_id_2 = -1; // Pick the best
  args->input_file_name = NULL;
  args->to_fix = NULL;
  args->rnti_to_fix = NULL;
  args->disable_cfo = false; 
  args->time_offset = 0; 
  args->file_nof_prb = 25; 
  args->file_nof_ports = 1; 
  args->file_cell_id = 0; 
  args->file_offset_time = 0; 
  args->file_offset_freq = 0; 
  args->rf_args = "";
  args->rf_freq = -1.0;
  args->rf_gain = -1.0; 
  args->net_port = -1; 
  args->net_address = "127.0.0.1";
  args->net_port_signal = -1; 
  args->net_address_signal = "127.0.0.1";
}

void usage(prog_args_t *args, char *prog) {
  printf("Usage: %s [agpPoOcildDnruvz] -f rx_frequency (in Hz) | -i input_file\n", prog);
#ifndef DISABLE_RF
  printf("\t-a RF args [Default %s]\n", args->rf_args);
  printf("\t-g RF fix RX gain [Default AGC]\n");
#else
  printf("\t   RF is disabled.\n");
#endif
  printf("\t-i input_file [Default use RF board]\n");
  printf("\t-o offset frequency correction (in Hz) for input file [Default %.1f Hz]\n", args->file_offset_freq);
  printf("\t-O offset samples for input file [Default %d]\n", args->file_offset_time);
  printf("\t-p nof_prb for input file [Default %d]\n", args->file_nof_prb);
  printf("\t-P nof_ports for input file [Default %d]\n", args->file_nof_ports);
  printf("\t-c cell_id for input file [Default %d]\n", args->file_cell_id);
  printf("\t-r RNTI in Hex [Default 0x%x]\n",args->rnti);
  printf("\t-l Force N_id_2 [Default best]\n");
  printf("\t-C Disable CFO correction [Default %s]\n", args->disable_cfo?"Disabled":"Enabled");
  printf("\t-t Add time offset [Default %d]\n", args->time_offset);
#ifndef DISABLE_GRAPHICS
  printf("\t-d disable plots [Default enabled]\n");
  printf("\t-D disable all but constellation plots [Default enabled]\n");
#else
  printf("\t plots are disabled. Graphics library not available\n");
#endif
  printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
  printf("\t-s remote UDP port to send input signal (-1 does nothing with it) [Default %d]\n", args->net_port_signal);
  printf("\t-S remote UDP address to send input signal [Default %s]\n", args->net_address_signal);
  printf("\t-u remote TCP port to send data (-1 does nothing with it) [Default %d]\n", args->net_port);
  printf("\t-U remote TCP address to send data [Default %s]\n", args->net_address);
  printf("\t-v [set srslte_verbose to debug, default none]\n");
  printf("\t-z filename reporting one location to be checked per line\n");
  printf("\t-Z filename reporting one rnti to be checked per line\n");
}

void parse_args(prog_args_t *args, int argc, char **argv) {
  int opt;
  args_default(args);
  while ((opt = getopt(argc, argv, "aoglipPcOCtdDnvrfuUsSzZ")) != -1) {
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
    case 'r':
      args->rnti = strtol(argv[optind], NULL, 16);
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
    case 'd':
      args->disable_plots = true;
      break;
    case 'D':
      args->disable_plots_except_constellation = true;
      break;
    case 'v':
      srslte_verbose++;
      break;
    case 'z':
      args->to_fix = argv[optind];
      break;
    case 'Z':
	  args->rnti_to_fix = argv[optind];
	  break;
	default:
      usage(args, argv[0]);
      exit(-1);
    }
  }
  if (args->input_file_name == NULL && args->to_fix == NULL) {
    usage(args, argv[0]);
    exit(-1);
  }
  if (args->rnti_to_fix == NULL) {
	  printf("### no RNTI file set, working as fix2 ###\n");
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

extern float mean_exec_time;

srslte_ue_dl_t ue_dl; 
srslte_ue_sync_t ue_sync; 
prog_args_t prog_args; 

uint32_t sfn = 0; // system frame number
cf_t *sf_buffer = NULL; 
srslte_netsink_t net_sink, net_sink_signal; 

int main(int argc, char **argv) {
  int ret; 
  srslte_cell_t cell;
  srslte_ue_mib_t ue_mib; 
#ifndef DISABLE_RF
  srslte_rf_t rf; 
#endif
  int n; 
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  int sfn_offset;
  uint32_t sfn_target = 0;
  uint32_t sf_target = 0;
  uint32_t ncce_target = 0;
  uint32_t L_target = 0;
  uint32_t cfi_target = 0;
  FILE *fid;
  uint16_t rnti_tmp;

  parse_args(&prog_args, argc, argv);
  
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

  }

  if (srslte_ue_mib_init(&ue_mib, cell)) {
    fprintf(stderr, "Error initaiting UE MIB decoder\n");
    exit(-1);
  }    

  if (srslte_ue_dl_init(&ue_dl, cell)) {  // This is the User RNTI
    fprintf(stderr, "Error initiating UE downlink processing module\n");
    exit(-1);
  }
  
  /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
  srslte_ue_dl_set_rnti(&ue_dl, prog_args.rnti);

  /* Initialize subframe counter */

  if (prog_args.rnti_to_fix != NULL) {
  	  fid = fopen(prog_args.rnti_to_fix,"r");
  	  for (int i=0;i<65536;i++) {
  	  	if (fscanf(fid,"%d",&rnti_tmp) != EOF) {
  //		  if (rnti_tmp) printf("rnti %d val %d\n", i, rnti_tmp);
  	  		srslte_ue_dl_reset_rnti_user_to(&ue_dl, i, rnti_tmp); // check this
  		  //printf("is rnti in the list? %d\n",rnti_in_list(&ue_dl, rnti_tmp));
  	  	}
  	  }
  	  fclose(fid);
    }

  ue_sync.correct_cfo = !prog_args.disable_cfo;

  srslte_pbch_decode_reset(&ue_mib.pbch);

  INFO("\nEntering main loop...\n\n", 0);
  /* Main loop */
  ret = srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
  if (ret < 0) {
      fprintf(stderr, "Error calling srslte_ue_sync_work()\n");
  }
  if (ret == 7) {
	  go_exit = true;
  }
  if (ret == 1) {
	if (srslte_ue_sync_get_sfidx(&ue_sync) == 0) {
    	n = srslte_ue_mib_decode(&ue_mib, sf_buffer, bch_payload, NULL, &sfn_offset);
        if (n < 0) {
        	fprintf(stderr, "Error decoding UE MIB\n");
            exit(-1);
        } else if (n == SRSLTE_UE_MIB_FOUND) {
        	srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
        	//srslte_cell_fprint(stdout, &cell, sfn);
        	//printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
        	sfn = (sfn + sfn_offset)%1024;
		}
	}
  } // Main loop
#define M_OFF 500
  float tprob[M_OFF];
  float bprob = 0;
  int b_offset = 0, b_offset2 = 0;
  int i, j, k;
  int t_offset, l_offset, m_offset = 30*ue_sync.sf_len/11520;
  uint16_t rnti_cand[M_OFF];
  uint16_t rnti_cnt[M_OFF][2];
  int rnti_off[M_OFF];
  uint16_t bcnt = 0;
  float rnti_bprob[M_OFF];
  int found = 0;

  fid = fopen(prog_args.to_fix,"r");
  bzero(tprob, sizeof(tprob));

  while (fscanf(fid,"%d %d %d %d %d",&sfn_target, &sf_target, &ncce_target, &L_target, &cfi_target) != EOF) {
//	  printf("checking sfn %d sf %d, ncce %d, L %d \n",sfn_target,sf_target, ncce_target, L_target);
	  bprob = 0;
	  b_offset = 0;
	  b_offset2 = 0;
	  bcnt = 0;
	  found = 0;
	  if (sfn == sfn_target && sf_target == 0) {
		  l_offset = 0;
	  } else {
		  srslte_filesource_seek(&ue_sync.file_source, ((sfn_target-sfn)*10+sf_target)*ue_sync.sf_len-m_offset);
		  srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
		  l_offset = -m_offset;
	  }
	  for (i=0; i<m_offset+1; i++) {
		  for (k=0; k<2 && !found; k++) {
			  if ((i==0 && k==1) || (l_offset==0 && k==1)) {
			  } else {
				  if (k==0) t_offset = i;
				  if (k==1) t_offset = -i;
				  //printf("checking offset %d for sfn %d sf %d (%d.%d)\n",t_offset,sfn_target,sf_target,i,k);
				  tprob[t_offset-l_offset] = srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[t_offset-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 0);
				  rnti_cand[t_offset-l_offset] = ue_dl.current_rnti;
				  rnti_cnt[t_offset-l_offset][0] = rnti_cand[t_offset-l_offset];
				  rnti_cnt[t_offset-l_offset][1] = 1;
				  rnti_off[t_offset-l_offset] = t_offset;
				  rnti_bprob[t_offset-l_offset] = tprob[t_offset-l_offset];

				  if ((tprob[t_offset-l_offset] >= 97 && tprob[t_offset-l_offset] <=101) || tprob[t_offset-l_offset] >= 197) {
					  //printf("Early winner...\n");
					  srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[t_offset-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 1);
					  found = 1;
					  break;
				  }

				  for (j = 0; j < t_offset-l_offset; j++) {
					  if (rnti_cnt[j][0] == rnti_cnt[t_offset-l_offset][0] && rnti_bprob[t_offset-l_offset] > 90) {
						  rnti_cnt[j][1]++;
						  //printf("at offset %d counting rnti %x cnt %d prob %.3f\n", t_offset, rnti_cnt[j][0], rnti_cnt[j][1], rnti_bprob[j]);
						  if (rnti_bprob[j] < rnti_bprob[t_offset-l_offset]) {
							  rnti_bprob[j] = rnti_bprob[t_offset-l_offset];
							  rnti_off[j] = t_offset;
							  //printf("at offset %d updating rnti %x cnt %d prob %.3f\n", t_offset, rnti_cnt[j][0], rnti_cnt[j][1], rnti_bprob[j]);
						  }
					  }
				  }
			  }
		  }
	  }
//	  for (t_offset = l_offset; t_offset < (m_offset+1); t_offset++) {
//		  //printf("checking offset %d for sfn %d sf %d\n",t_offset,sfn_target,sf_target);
//		  tprob[t_offset-l_offset] = srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[t_offset-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 0);
//		  rnti_cand[t_offset-l_offset] = ue_dl.current_rnti;
//		  rnti_cnt[t_offset-l_offset][0] = rnti_cand[t_offset-l_offset];
//		  rnti_cnt[t_offset-l_offset][1] = 1;
//		  rnti_off[t_offset-l_offset] = t_offset;
//		  rnti_bprob[t_offset-l_offset] = tprob[t_offset-l_offset];
//
//		  if ((tprob[t_offset-l_offset] >= 97 && tprob[t_offset-l_offset] <=101) || tprob[t_offset-l_offset] >= 197) {
//			  //printf("Early winner...\n");
//			  srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[t_offset-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 1);
//			  found = 1;
//			  break;
//		  }
//
//		  for (j = 0; j < t_offset-l_offset; j++) {
//			  if (rnti_cnt[j][0] == rnti_cnt[t_offset-l_offset][0] && rnti_bprob[t_offset-l_offset] > 90) {
//				  rnti_cnt[j][1]++;
//				  //printf("at offset %d counting rnti %x cnt %d prob %.3f\n", t_offset, rnti_cnt[j][0], rnti_cnt[j][1], rnti_bprob[j]);
//				  if (rnti_bprob[j] < rnti_bprob[t_offset-l_offset]) {
//					  rnti_bprob[j] = rnti_bprob[t_offset-l_offset];
//					  rnti_off[j] = t_offset;
//					  //printf("at offset %d updating rnti %x cnt %d prob %.3f\n", t_offset, rnti_cnt[j][0], rnti_cnt[j][1], rnti_bprob[j]);
//				  }
//			  }
//		  }
//	  }
	  if (found == 0) {
		  for (j = 0; j < (m_offset*2+1); j++) {
			  //if (rnti_cnt[j][1] > 1 && rnti_bprob[j] > 90) printf("entry %d, rnti %x cnt %d prob %.3f offset %d\n", j, rnti_cnt[j][0], rnti_cnt[j][1], rnti_bprob[j], rnti_off[j]);
			  if (rnti_cnt[j][1] > bcnt) {
				  bcnt = rnti_cnt[j][1];
				  b_offset = rnti_off[j];
			  }
			  if (rnti_cnt[j][1] > 1 && rnti_bprob[j] > bprob) {
				  bprob = rnti_bprob[j];
				  b_offset2 = rnti_off[j];
			  }
		  }
		  if (rnti_cand[b_offset-l_offset] == rnti_cand[b_offset2-l_offset]) {
			  if (rnti_bprob[b_offset-l_offset] >= rnti_bprob[b_offset2-l_offset] && rnti_bprob[b_offset-l_offset] >= 97) {
				  //printf("Most popular at offset %d...\n",b_offset);
				  //srslte_filesource_seek(&ue_sync.file_source, ((sfn_target-sfn)*10+sf_target)*ue_sync.sf_len+b_offset);
				  //srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
				  srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[b_offset-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 1);
			  } else if (rnti_bprob[b_offset2-l_offset] >= 97) {
				  //printf("Most likely not unique at offset %d...\n",b_offset2);
				  //srslte_filesource_seek(&ue_sync.file_source, ((sfn_target-sfn)*10+sf_target)*ue_sync.sf_len+b_offset2);
				  //srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
				  srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[b_offset2-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 1);
			  }
		  } else {
			  //printf("Most popular at offset %d...\n",b_offset);
			  //srslte_filesource_seek(&ue_sync.file_source, ((sfn_target-sfn)*10+sf_target)*ue_sync.sf_len+b_offset);
			  //srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
			  if (rnti_bprob[b_offset-l_offset] >= 95) srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[b_offset-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 1);
			  //printf("Most likely not unique at offset %d...\n",b_offset2);
			  //srslte_filesource_seek(&ue_sync.file_source, ((sfn_target-sfn)*10+sf_target)*ue_sync.sf_len+b_offset2);
			  //srslte_ue_sync_get_buffer(&ue_sync, &sf_buffer);
			  if (rnti_bprob[b_offset2-l_offset] >= 95) srslte_ue_dl_fix_control_ra(&ue_dl, &sf_buffer[b_offset2-l_offset], data, sf_target, 0, sfn_target, ncce_target, L_target, cfi_target, 1);

			  //if (rnti_bprob[b_offset2-l_offset] < 98 && rnti_bprob[b_offset-l_offset] < 98) printf("nothing found in %d.%d L=%d,%d,%d\n",sfn_target,sf_target,ncce_target, L_target, cfi_target);
		  }
	  }
	  //if (sfn_target > 0) exit(0);
  }
  srslte_ue_dl_free(&ue_dl);
  srslte_ue_sync_free(&ue_sync);
  
#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_ue_mib_free(&ue_mib);
    srslte_rf_close(&rf);    
  }
#endif
  //printf("\nBye\n");
  exit(0);
}
