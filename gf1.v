module gf1
	(
	input MCLK,
	input CLK, // div 2 (9878400 hz),
	input IOW,
	input IOR,
	input CS1, // GF1
	input CS2, // MIDI
	input DACK1, // DRAM
	input DACK2, // RECORD
	input RESET,
	output WAIT,
	output DRQ1,
	output DRQ2,
	output IRQ1,
	output IRQ2,
	output IRQ3,
	input DMA_TC,
	input [15:0] DATA_i,
	output [15:0] DATA_o,
	input IO16,
	input [3:0] ADDRESS,
	output BUSA_LOW,
	output BUSA_HIGH,
	input [7:0] DRAM_DATA_i,
	output [7:0] DRAM_DATA_o,
	output reg DRAM_WE,
	output reg DRAM_RAS,
	output [8:0] DRAM_ADDR,
	output DRAM_CAS0,
	output DRAM_CAS1,
	output DRAM_CAS2,
	output DRAM_CAS3,
	output DAC_DATA,
	output reg DAC_CLK,
	output reg DAC_LR,
	output reg DAC_LR2
	);

	reg clk_sel[0:16];
	reg clk1, clk2, clk3, clk4;
	reg chan_sel_i;
	reg [5:0] chan_cnt[0:1];
	
	reg chan_cnt_res_l;
	reg chan_c0_l;
	reg [4:0] chan_latch1[0:3];
	reg [4:0] chan_latch2;
	
	reg [4:0] chan_active_cnt;
	reg [4:0] chan_active_cnt_l;
	reg [4:0] chan_irq;
	reg [4:0] chan_irq_l;
	reg [4:0] chan_cmp_l;
	reg [4:0] chan_cmp_l2;
	reg [4:0] ram_addr_l;
	reg chan_cnt_res;
	reg clk4_chc0;
	reg clk4_chc0n;
	
	reg voice_ram_update_pipeline;
	reg voice_ram_update;
	
	reg [4:0] chan_sel;
	
	reg [15:0] glob_data_bus;
	
	wire chan_c0 = chan_cnt[1][0];
	
	wire chan_active_match = chan_cmp_l == chan_active_cnt_l;
	wire chan_irq_match = chan_cmp_l2 == chan_irq_l;
	
	reg [147:0] voice_ram[0:31];
	reg [147:0] ram_right_bus;
	reg [147:0] ram_left_bus;
	reg [147:0] ram_output_latch;
	reg [147:0] ram_input_latch;
	
	wire [7:0] wave_params = ram_output_latch[147:140];
	reg [5:0] wave_params_l1[0:2];
	reg [2:0] wave_params_l2[0:2];
	reg wave_roll[0:2];
	reg wave_end_reach;
	
	// 0 - status
	// 1 - loop
	// 2 - bidir
	// 3 - irq en
	// 4 - dir
	// 5 - irq pend
	
	wire wave_end_cond = wave_end_reach & (wave_params_l1[2][1] | ~wave_roll[2]);
	wire wave_irq_pend_next = wave_params_l1[2][3] & (wave_params_l1[2][5] | wave_end_reach);
	wire wave_status_next = wave_params_l1[2][0] | (~wave_params_l1[2][0] & (wave_end_cond & ~wave_params_l1[2][1]));
	wire wave_dir_next = wave_params_l1[2][4] ^ (wave_end_cond & (wave_params_l1[2][1] & wave_params_l1[2][2]));
	
	wire wave_is_stopped = wave_params[1] | wave_params[0];
	
	reg w1811;
	reg w1856;
	reg w1813;
	reg w1853;
	reg wave_phase1;
	reg wave_phase2;
	reg wave_phase3;
	reg wave_phase4;
	reg wave_of_check_l;
	
	reg [28:0] wave_mux_lo;
	reg [28:0] wave_mux_hi;
	reg [28:0] wave_inc2;
	reg wave_sel1; // w1867
	reg wave_sel2; // w1818
	reg wave_sel3; // w173
	
	wire [28:0] wave_add1 = (wave_sel2 ? wave_inc2 : (wave_sel1 ? wave_mux_lo : wave_mux_hi)) ^ {29{wave_sel3}};
	wire [28:0] wave_add2 = wave_sel1 ? wave_mux_hi : wave_mux_hi;
	wire [29:0] wave_sum = { 1'h0, wave_add1 } + { 1'h0, wave_add2 } + { 29'h0, wave_sel3 };
	reg [28:0] wave_sum_mem;
	
	reg [28:0] wave_next_l[0:5];
	reg [28:0] wave_addr_l[0:2];
	
	
	reg wave_irq_set;
	reg wave_irq_set_l;
	reg wave_irq_clear;
	reg wave_irq_clear_l1;
	reg wave_irq_clear_l2;
	reg wave_irq_clear_l3;
	reg wave_irq_clear_l4;
	reg wave_irq_clear_l5;
	reg wave_irq_set_cond;
	
	
	wire [28:0] wave_cur = {
			ram_output_latch[139],
			ram_output_latch[136],
			ram_output_latch[133],
			ram_output_latch[130],
			ram_output_latch[127],
			ram_output_latch[124],
			ram_output_latch[121],
			ram_output_latch[118],
			ram_output_latch[115],
			ram_output_latch[112],
			ram_output_latch[109],
			ram_output_latch[106],
			ram_output_latch[103],
			ram_output_latch[100],
			ram_output_latch[97],
			ram_output_latch[93],
			ram_output_latch[89],
			ram_output_latch[85],
			ram_output_latch[81],
			ram_output_latch[77],
			ram_output_latch[73],
			ram_output_latch[69],
			ram_output_latch[65],
			ram_output_latch[61],
			ram_output_latch[57],
			ram_output_latch[55],
			ram_output_latch[53],
			ram_output_latch[51],
			ram_output_latch[49]
		};
	
	wire [28:0] wave_start = {
			ram_output_latch[138],
			ram_output_latch[135],
			ram_output_latch[132],
			ram_output_latch[129],
			ram_output_latch[128],
			ram_output_latch[123],
			ram_output_latch[120],
			ram_output_latch[117],
			ram_output_latch[114],
			ram_output_latch[111],
			ram_output_latch[108],
			ram_output_latch[105],
			ram_output_latch[102],
			ram_output_latch[99],
			ram_output_latch[96],
			ram_output_latch[92],
			ram_output_latch[88],
			ram_output_latch[84],
			ram_output_latch[80],
			ram_output_latch[76],
			ram_output_latch[72],
			ram_output_latch[68],
			ram_output_latch[64],
			ram_output_latch[60],
			5'h0
		};
	
	wire [28:0] wave_end = {
			ram_output_latch[137],
			ram_output_latch[134],
			ram_output_latch[131],
			ram_output_latch[128],
			ram_output_latch[127],
			ram_output_latch[122],
			ram_output_latch[119],
			ram_output_latch[116],
			ram_output_latch[113],
			ram_output_latch[110],
			ram_output_latch[107],
			ram_output_latch[104],
			ram_output_latch[101],
			ram_output_latch[98],
			ram_output_latch[95],
			ram_output_latch[91],
			ram_output_latch[87],
			ram_output_latch[83],
			ram_output_latch[79],
			ram_output_latch[75],
			ram_output_latch[71],
			ram_output_latch[67],
			ram_output_latch[63],
			ram_output_latch[59],
			5'h0
		};
	
	wire [28:0] wave_inc = {
			14'h0,
			ram_output_latch[94],
			ram_output_latch[90],
			ram_output_latch[86],
			ram_output_latch[82],
			ram_output_latch[78],
			ram_output_latch[74],
			ram_output_latch[70],
			ram_output_latch[66],
			ram_output_latch[62],
			ram_output_latch[58],
			ram_output_latch[56],
			ram_output_latch[54],
			ram_output_latch[52],
			ram_output_latch[50],
			ram_output_latch[48]
		};
	
	wire [7:0] ramp_params = ram_output_latch[41:34];
	reg [5:0] ramp_params_l1[0:2];
	reg [2:0] ramp_params_l2[0:2];
	reg ramp_end_reach;
	
	// 0 - status
	// 1 - loop
	// 2 - bidir
	// 3 - irq en
	// 4 - dir
	// 5 - irq pend
	
	wire ramp_end_cond = ramp_end_reach;
	wire ramp_irq_pend_next = ramp_params_l1[2][3] & (ramp_params_l1[2][5] | ramp_end_reach);
	wire ramp_status_next = ramp_params_l1[2][0] | (~ramp_params_l1[2][0] & (ramp_end_cond & ~ramp_params_l1[2][1]));
	wire ramp_dir_next = ramp_params_l1[2][4] ^ (ramp_end_cond & (ramp_params_l1[2][1] & ramp_params_l1[2][2]));
	
	wire ramp_is_stopped = ramp_params[1] | ramp_params[0];
	
	reg w1935;
	reg w1987;
	reg w1989;
	reg w1932;
	reg ramp_phase1;
	reg ramp_phase2;
	reg ramp_phase3;
	reg ramp_phase4;
	reg ramp_of_check_l;
	
	reg [11:0] ramp_mux_lo;
	reg [11:0] ramp_mux_hi;
	reg ramp_sel1;
	reg ramp_sel2;
	reg ramp_sel3;
	
	wire [11:0] ramp_add1 = (ramp_sel2 ? 12'h0 : (ramp_sel1 ? ramp_mux_lo : ramp_mux_hi)) ^ {12{ramp_sel3}};
	wire [11:0] ramp_add2 = ramp_sel1 ? ramp_mux_hi : ramp_mux_hi;
	wire [12:0] ramp_sum = { 1'h0, ramp_add1 } + { 1'h0, ramp_add2 } + { 12'h0, ramp_sel3 };
	reg [11:0] ramp_sum_mem;
	
	reg [11:0] ramp_next_l[0:5];
	reg [11:0] ramp_addr_l[0:2];
	
	reg [8:0] ramp_cnt[0:1];
	reg [4:0] ramp_ch_l;
	reg w368;
	
	reg ramp_irq_set;
	reg ramp_irq_set_l;
	reg ramp_irq_clear;
	reg ramp_irq_clear_l1;
	reg ramp_irq_clear_l2;
	reg ramp_irq_clear_l3;
	reg ramp_irq_clear_l4;
	reg ramp_irq_clear_l5;
	reg ramp_irq_set_cond;
	
	wire [11:0] ramp_cur = {
			ram_output_latch[33],
			ram_output_latch[30],
			ram_output_latch[27],
			ram_output_latch[24],
			ram_output_latch[21],
			ram_output_latch[18],
			ram_output_latch[15],
			ram_output_latch[11],
			ram_output_latch[7],
			ram_output_latch[5],
			ram_output_latch[3],
			ram_output_latch[1]
		};
		
	wire [11:0] ramp_start = {
			ram_output_latch[32],
			ram_output_latch[29],
			ram_output_latch[26],
			ram_output_latch[23],
			ram_output_latch[20],
			ram_output_latch[17],
			ram_output_latch[14],
			ram_output_latch[10],
			4'h0
		};
		
	wire [11:0] ramp_end = {
			ram_output_latch[31],
			ram_output_latch[28],
			ram_output_latch[25],
			ram_output_latch[22],
			ram_output_latch[19],
			ram_output_latch[16],
			ram_output_latch[13],
			ram_output_latch[9],
			4'h0
		};
		
	wire [11:0] ramp_inc = {
			6'h0,
			ram_output_latch[12] & w368,
			ram_output_latch[8] & w368,
			ram_output_latch[6] & w368,
			ram_output_latch[4] & w368,
			ram_output_latch[2] & w368,
			ram_output_latch[0] & w368
		};
	
	reg voice_irq_read_l;
	reg voice_irq_read_l2;
	
	reg [15:0] cpu_bus;
	
	// stubs
	
	wire rec_dma_chan_width = 0; // stub
	wire midi_rx_irq = 0;
	wire midi_tx_irq = 0;
	
	assign DRQ2 = 0;
	assign IRQ3 = 0;
	
	//
	
	
	wire dack_comb = DACK1 | DACK2;
	
	wire cpu_write = IOW & ~dack_comb;
	wire cpu_read = IOR & ~dack_comb;
	wire cpu_addr_0 = ADDRESS == 4'h0;
	wire cpu_addr_1 = ADDRESS == 4'h1;
	wire cpu_addr_2 = ADDRESS == 4'h2;
	wire cpu_addr_3 = ADDRESS == 4'h3;
	wire cpu_addr_4 = ADDRESS == 4'h4;
	wire cpu_addr_5 = ADDRESS == 4'h5;
	wire cpu_addr_6 = ADDRESS == 4'h6;
	wire cpu_addr_7 = ADDRESS == 4'h7;
	wire cpu_addr_8 = ADDRESS == 4'h8;
	wire cpu_addr_9 = ADDRESS == 4'h9;
	wire cpu_addr_A = ADDRESS == 4'hA;
	wire cpu_addr_C = ADDRESS == 4'hC;
	wire cpu_addr_D = ADDRESS == 4'hD;
	wire cpu_addr_E = ADDRESS == 4'hE;
	
	wire cpu_input = IOW & (dack_comb |
		(CS1 & (cpu_addr_1 | cpu_addr_2 | cpu_addr_3 | cpu_addr_4 | cpu_addr_5 | cpu_addr_6 | cpu_addr_7
			| cpu_addr_8 | cpu_addr_9 | cpu_addr_A | cpu_addr_C | cpu_addr_D | cpu_addr_E)) |
			(CS2 & (cpu_addr_0 | cpu_addr_1)));
	
	wire cpu_output = IOR & (dack_comb |
		(CS1 & (cpu_addr_1 | cpu_addr_2 | cpu_addr_3 | cpu_addr_4 | cpu_addr_5 | cpu_addr_6 | cpu_addr_7
			| cpu_addr_8 | cpu_addr_9 | cpu_addr_A | cpu_addr_C | cpu_addr_E)) |
			(CS2 & (cpu_addr_0 | cpu_addr_1)));
	
	assign DATA_o = cpu_bus;
	
	reg [7:0] glob_reg;
	reg [7:0] glob_reg_l2;
	reg [15:0] glob_data_read;
	reg [7:0] glob_data_l1;
	reg [15:0] glob_data_l2;
	wire glob_data_exec = cpu_write & CS1 & ((cpu_addr_4 & IO16) | cpu_addr_5);
	reg glob_data_wr1;
	reg glob_data_wr2;
	reg glob_data_rd1;
	reg glob_data_rd2;
	reg glob_data_rd3;
	
	wire cpu_write_4 = cpu_write & CS1 & cpu_addr_4;
	wire cpu_write_5 = cpu_write & CS1 & cpu_addr_5;
	
	wire glob_addr_41 = glob_reg_l2[6:0] == 7'h41;
	wire glob_addr_42 = glob_reg_l2[6:0] == 7'h42;
	wire glob_addr_43 = glob_reg_l2[6:0] == 7'h43;
	wire glob_addr_44 = glob_reg_l2[6:0] == 7'h44;
	wire glob_addr_45 = glob_reg_l2[6:0] == 7'h45;
	wire glob_addr_46 = glob_reg_l2[6:0] == 7'h46;
	wire glob_addr_47 = glob_reg_l2[6:0] == 7'h47;
	wire glob_addr_48 = glob_reg_l2[6:0] == 7'h48;
	wire glob_addr_49 = glob_reg_l2[6:0] == 7'h49;
	wire glob_addr_4b = glob_reg_l2[6:0] == 7'h4b;
	wire glob_addr_4c = glob_reg_l2[6:0] == 7'h4c;
	
	reg [7:0] glob_read_mux;
	
	reg dram_dma_invert_msb;
	reg dram_dma_data_size;
	reg dram_dma_irq_en_l;
	reg dram_dma_irq_en;
	reg dram_dma_rate1;
	reg dram_dma_rate0;
	reg dram_dma_chan_width;
	reg dram_dma_dir;
	reg dram_dma_en_l;
	reg dram_dma_en;
	
	reg [19:0] dram_dma_address[0:1];
	reg [19:0] dram_peek_address;
	
	reg timer_ctrl_b5_l;
	reg timer_ctrl_b5;
	reg timer_ctrl_b4_l;
	reg timer_ctrl_b4;
	reg timer2_irq_en_l;
	reg timer2_irq_en;
	reg timer1_irq_en_l;
	reg timer1_irq_en;
	reg timer_ctrl_b1_l;
	reg timer_ctrl_b1;
	reg timer_ctrl_b0_l;
	reg timer_ctrl_b0;
	
	reg [7:0] timer1_reg_l;
	reg [7:0] timer1_reg;
	reg [7:0] timer2_reg_l;
	reg [7:0] timer2_reg;
	
	reg clk_div_reset[0:1];
	reg [3:0] clk_div_cnt[0:1];
	reg clk_div_reset2[0:1];
	reg [6:0] clk_div_cnt2[0:1];
	reg [1:0] clk_div_cnt3[0:1];
	reg clk_div_timer;
	reg clk_div_timer2;
	
	reg [7:0] timer1_cnt[0:1];
	wire [8:0] timer1_cnt_add = { 1'h0, timer1_cnt[1] } + 9'h1;
	wire timer1_of = timer1_cnt_add[8];
	reg timer1_irq_l;
	reg timer1_irq;
	reg timer1_expire_l;
	reg timer1_expire;
	reg timer1_start_l;
	reg timer1_start;
	reg timer1_mask_l;
	reg timer1_mask;
	reg [7:0] timer2_cnt[0:1];
	wire [8:0] timer2_cnt_add = { 1'h0, timer2_cnt[1] } + 9'h1;
	wire timer2_of = timer2_cnt_add[8];
	reg timer2_irq_l;
	reg timer2_irq;
	reg timer2_expire_l;
	reg timer2_expire;
	reg timer2_start_l;
	reg timer2_start;
	reg timer2_mask_l;
	reg timer2_mask;
	reg timer_irq_res_l;
	reg timer_irq_res;
	
	reg adlib_io_req_l;
	reg adlib_io_req;
	reg blaster_io_dsp_req_l;
	reg blaster_io_dsp_req;
	reg blaster_io_dsp_reset_req_l;
	reg blaster_io_dsp_reset_req;
	
	reg voice_write_pending[0:1];
	reg voice_read_pending[0:1];
	wire voice_rw_pending = voice_write_pending[1] | voice_read_pending[1];
	
	wire cpu_write3 = cpu_write & CS1 & cpu_addr_3;
	
	reg gf1_irq_enable_l;
	reg gf1_irq_enable;
	reg dac_enable_l;
	reg dac_enable;
	reg reset_reg_not_l;
	reg reset_reg_not;
	
	wire wave_irq = gf1_irq_enable & wave_irq_set;
	wire ramp_irq = gf1_irq_enable & ramp_irq_set;
	
	wire reset_reg = ~reset_reg_not;
	
	reg [7:0] peek_reg;
	
	reg [7:0] reg_2XA;
	reg [7:0] reg_2XC;
	reg [7:0] reg_2XE;
	reg [7:0] adlib_reg;
	reg [7:0] adlib_data;
	
	reg adlib_reg_sel_4[0:1];
	
	reg dram_dma_tc_l;
	reg dram_dma_tc;
	reg dram_dma_irq_pending;
	
	reg dram_dma_req_wr1;
	reg dram_dma_req_wr2;
	reg dram_dma_req_rd;
	
	reg cls9_dma_write;
	reg cls9_dma_read;
	
	reg dram_refresh_slot;
	
	reg [2:0] dram_dma_clk_cnt[0:1];
	
	wire dram_dma_rate00 = ~dram_dma_rate0 & ~dram_dma_rate1;
	wire dram_dma_rate01 = ~dram_dma_rate0 & dram_dma_rate1;
	wire dram_dma_rate10 = dram_dma_rate0 & ~dram_dma_rate1;
	wire dram_dma_rate11 = dram_dma_rate0 & dram_dma_rate1;
	
	wire [3:0] dram_dma_clk_cntc = { dram_dma_clk_cnt[1], ~dram_refresh_slot };
	
	wire dram_dma_tick = dram_dma_en & (
		(dram_dma_rate00 & 1'h1) |
		(dram_dma_rate01 & dram_dma_clk_cntc[1:0] == 2'h0) |
		(dram_dma_rate10 & dram_dma_clk_cntc[2:0] == 3'h0) |
		(dram_dma_rate11 & dram_dma_clk_cntc[3:0] == 4'h0));
	
	assign DRQ1 = (~dram_dma_tc & dram_dma_req_wr2) | dram_dma_req_rd;
	
	reg [15:0] dram_dma_in_data;
	wire [15:0] dram_dma_in_data_sign;
	
	reg dram_dma_hi_lo_byte_sel[0:1];
	
	assign dram_dma_in_data_sign[6:0] = dram_dma_in_data[6:0];
	assign dram_dma_in_data_sign[14:8] = dram_dma_in_data[14:8];
	assign dram_dma_in_data_sign[15] = dram_dma_in_data[15] ^ (dram_dma_chan_width & dram_dma_invert_msb);
	
	assign dram_dma_in_data_sign[7] = dram_dma_in_data[7] ^ (dram_dma_invert_msb &
		(~dram_dma_data_size | (~dram_dma_hi_lo_byte_sel[1] & dram_dma_data_size & ~dram_dma_chan_width)));
	
	reg dram_dma_write_state1;
	reg dram_dma_write_state2;
	
	wire dram_dma_irq_ack = cpu_read & CS1 & cpu_addr_5 & glob_addr_41;
	
	reg dram_io_wr_dma_l;
	reg dram_io_wr_dma;
	reg dram_io_rd_dma_l;
	reg dram_io_rd_dma;
	reg dram_io_pp_l;
	reg dram_io_pp;
	
	wire dram_dma_access = dram_io_wr_dma | dram_io_rd_dma;
	
	reg [15:0] dram_dma_rd_latch;
	
	reg dram_dma_read_state1;
	reg dram_dma_read_state2;
	reg dram_dma_read_state3;
	
	wire dram_cpu_poke = cpu_write & CS1 & cpu_addr_7;
	wire dram_cpu_peek = cpu_read & CS1 & cpu_addr_7;
	wire dram_cpu_pp = dram_cpu_poke | dram_cpu_peek;
	wire dram_pp_enable = dram_io_pp & ~dram_io_wr_dma & (~dram_io_rd_dma | dram_io_wr_dma);
	
	reg dram_pp_state;
	
	wire dram_pp_wait = ~reset_reg & (dram_cpu_pp & ~dram_pp_state);
	
	wire cpu_dram_access = dram_io_pp | dram_io_wr_dma | dram_io_rd_dma;
	
	wire dram_io_dir = dram_io_wr_dma | (dram_pp_enable & ~dram_cpu_peek);
	
	wire dram_cpu_access_16 = dram_dma_chan_width & (dram_io_rd_dma | dram_io_wr_dma);
	reg dram_voice_access_16;
	reg dram_access_16;
	reg dram_in_16;
	
	reg [15:0] dram_bus;
	reg [15:0] dram_in;
	
	reg dram_subaddr;
	reg dram_hi_sel;
	
	assign DRAM_DATA_o = ((dram_access_16 & ~dram_subaddr) | ~dram_access_16) ? dram_bus[7:0] : dram_bus[15:8];
	
	reg dram_addr_cw_sel;
	
	wire [19:0] dram_cpu_addr = dram_dma_access ? dram_dma_address[1] : dram_peek_address;
	wire [19:0] dram_addr_linear = dram_addr_cw_sel ? dram_cpu_addr : wave_addr_l[2][28:9];
	
	wire [10:0] dram_addr1 = dram_hi_sel ?
		{ dram_addr_linear[16], dram_addr_linear[15], dram_addr_linear[15], dram_addr_linear[14:7] } :
		{ dram_addr_linear[17], dram_addr_linear[16], dram_addr_linear[7:0], dram_subaddr };
	
	assign DRAM_ADDR = dram_access_16 ?
		{ dram_addr1[9], dram_addr1[7:0] } :
		{ dram_addr1[10], dram_addr1[8:1] };
	
	reg dram_cas;
	reg dram_cas_refresh;
	
	assign DRAM_CAS0 = dram_cas & (dram_addr_linear[19:18] == 2'h0 | dram_cas_refresh);
	assign DRAM_CAS1 = dram_cas & (dram_addr_linear[19:18] == 2'h1 | dram_cas_refresh);
	assign DRAM_CAS2 = dram_cas & (dram_addr_linear[19:18] == 2'h2 | dram_cas_refresh);
	assign DRAM_CAS3 = dram_cas & (dram_addr_linear[19:18] == 2'h3 | dram_cas_refresh);
	
	reg dram_latch_hi;
	reg dram_latch_lo;
	
	
	wire dma_tc_irq = dram_dma_irq_en & dram_dma_tc; // TODO: record dma tc
	
	reg [4:0] dest_channel;
	reg [4:0] dest_channel_l2;
	
	reg [5:0] voice_reg_sel;
	wire voice_reg_0 = voice_reg_sel[4:0] == 5'h0;
	wire voice_reg_1 = voice_reg_sel[4:0] == 5'h1;
	wire voice_reg_2 = voice_reg_sel[4:0] == 5'h2;
	wire voice_reg_3 = voice_reg_sel[4:0] == 5'h3;
	wire voice_reg_4 = voice_reg_sel[4:0] == 5'h4;
	wire voice_reg_5 = voice_reg_sel[4:0] == 5'h5;
	wire voice_reg_6 = voice_reg_sel[4:0] == 5'h6;
	wire voice_reg_7 = voice_reg_sel[4:0] == 5'h7;
	wire voice_reg_8 = voice_reg_sel[4:0] == 5'h8;
	wire voice_reg_9 = voice_reg_sel[4:0] == 5'h9;
	wire voice_reg_A = voice_reg_sel[4:0] == 5'ha;
	wire voice_reg_B = voice_reg_sel[4:0] == 5'hb;
	wire voice_reg_C = voice_reg_sel[4:0] == 5'hc;
	wire voice_reg_D = voice_reg_sel[4:0] == 5'hd;
	wire voice_reg_E = voice_reg_sel[3:0] == 4'he;
	wire voice_reg_F = voice_reg_sel[3:0] == 4'hf;
	
	wire blaster_io_irq = blaster_io_dsp_req | blaster_io_dsp_reset_req | adlib_io_req;
	wire adlib_irq = timer1_expire | timer2_expire;
	
	wire wait_voice_write = glob_data_exec & glob_data_wr2;
	wire wait_voice_read = (glob_data_rd3 & cpu_read & CS1 & (cpu_addr_4 | cpu_addr_5)) | (glob_data_wr2 & cpu_write3);
	
	assign WAIT = dram_pp_wait | wait_voice_read | wait_voice_write;
	
	reg ram_strobe;
	reg ram_wr_strobe;
	
	reg w1816;
	reg w1822;
	reg w1848;
	reg w1885;
	
	reg w1823;
	reg w1845;
	reg w192;
	
	reg w100;
	reg w134;
	reg w1863;
	reg w1982;
	
	reg w1800;
	reg w1801;
	reg w1944;
	
	reg w434;
	
	reg wave_is_stopped_l[0:4];
	reg ramp_is_stopped_l[0:4];
	
	reg voice_ram_read;
	
	reg [15:0] mul_val_a_l;
	reg [15:0] mul_val_a;
	reg [8:0] mul_val_b;
	
	reg [7:0] interp_in;
	reg [8:0] interp_in_mul;
	
	wire signed [15:0] mul_val_a_s = mul_val_a;
	wire signed [9:0] mul_val_b_s = { 1'h0, mul_val_b };
	reg signed [24:0] mul_result;
	
	reg [15:0] mul_mem[0:1];
	reg [15:0] mul_interp;
	
	reg [11:0] atten_l[0:1];
	reg [3:0] pan_l[0:1];
	reg w2048;
	wire [3:0] pan = w2048 ? pan_l[1] : ~pan_l[1];
	
	reg [8:0] pan_atten;
	
	wire [12:0] atten1 = { 1'h0, atten_l[1] } + { pan_atten, 4'h0 };
	wire [11:0] atten = ((pan == 4'hf) | atten1[12]) ? 12'hfff : atten1[11:0];
	
	reg [15:0] val_shifted;
	
	reg [20:0] accum_l[0:1];
	
	reg [20:0] accum_add;
	
	wire [20:0] accum_sum = accum_add + { {5{val_shifted[15]}}, val_shifted }; 
	
	wire accum_clip1 = ~accum_sum[20] & (accum_sum[19:15] != 5'h0);
	wire accum_clip2 = accum_sum[20] & (accum_sum[19:15] != 5'h1f);
	wire [15:0] accum_clip = accum_clip2 ? 16'h8000 : (accum_clip1 ? 16'h7fff : accum_sum[15:0]);
	reg [15:0] accum_mem;
	
	reg [15:0] accum_shifter[0:1];
	
	reg [5:0] dac_counter[0:1];
	reg w2008[0:1];
	reg w2007;
	
	reg w2003;
	
	wire w2042 = ~(dac_counter[1][1] & dac_counter[1][2]);
	wire w2045 = ~(~dac_counter[1][0] & ~dac_counter[1][1]);
	wire w2043 = ~(w2042 & dac_counter[1][5] & ~dac_counter[1][4] & ~dac_counter[1][3]);
	wire w2044 = ~(w2045 & dac_counter[1][4] & dac_counter[1][3] & dac_counter[1][2]);
	wire w2020 = ~(w2043 & w2044);
	wire w2029 = ~(~dac_counter[1][3] & dac_counter[1][2] & ~dac_counter[1][1] & dac_counter[1][0]);
	wire w2022 = ~(~dac_counter[1][4] & ~dac_counter[1][5]);
	wire w2021 = ~w2022;
	wire w1999 = ~(w2029 | w2022);
	wire w1998 = ~(w2020 | w1999);
	
	wire w2059 = ~(~dac_counter[1][5] & ~(dac_counter[1][4] & dac_counter[1][3] & dac_counter[1][2]));
	
	wire w2031 = ~(~dac_counter[1][3] | ~dac_counter[1][2] | ~dac_counter[1][1]);
	
	wire w2038 = ~(w2003 & (w1999 | w2022 | w2031));
	
	wire w2055 = ~(~dac_counter[1][5] | dac_counter[1][4] | ~dac_counter[1][3] | ~dac_counter[1][0]);
	wire w2040 = ~(dac_counter[1][5] | dac_counter[1][4] | dac_counter[1][3] | ~dac_counter[1][0]);
	
	assign DAC_DATA = accum_shifter[1][15];
	
	assign IRQ1 = dma_tc_irq | ramp_irq | wave_irq;
	assign IRQ2 = blaster_io_irq | timer2_irq | timer1_irq;
	
	assign BUSA_LOW = cpu_input | cpu_output;
	assign BUSA_HIGH = (cpu_input | cpu_output) & (IO16 | (DACK1 & dram_dma_chan_width) | (DACK2 & rec_dma_chan_width));
	
	reg w2056;
	reg w33;
	reg w2036;
	reg w2035;
	reg w2415;
	reg w2416;
	reg w1924;
	reg w1923;
	reg w2360;
	reg w2360_l[0:3];
	
	reg dac_clk1_l[0:1];
	reg dac_clk2_l[0:1];
	reg dac_clk3_l[0:1];
	reg dac_clk4_l[0:1];
	
	reg w262;
	reg w262_l[0:2];
	
	always @(posedge MCLK)
	begin
		if (!CLK)
		begin
        clk_sel[1] = clk_sel[0];
        clk_sel[3] = clk_sel[2];
        clk_sel[5] = clk_sel[4];
        clk_sel[7] = clk_sel[6];
        clk_sel[9] = clk_sel[8];
        clk_sel[11] = clk_sel[10];
        clk_sel[13] = clk_sel[12];
        clk_sel[15] = clk_sel[14];
		end
		else
		begin
        clk_sel[2] = clk_sel[1];
        clk_sel[4] = clk_sel[3];
        clk_sel[6] = clk_sel[5];
        clk_sel[8] = clk_sel[7];
        clk_sel[10] = clk_sel[9];
        clk_sel[12] = clk_sel[11];
        clk_sel[14] = clk_sel[13];
        clk_sel[16] = clk_sel[15];

        clk_sel[0] = ~(clk_sel[1] | clk_sel[3] | clk_sel[5] |
            clk_sel[7] | clk_sel[9] | clk_sel[11] | clk_sel[13]);
		end
		
		if (clk_sel[13])
			clk1 = 1;
		else if (clk_sel[1])
			clk1 = 0;
		if (clk_sel[1])
			clk2 = 1;
		else if (clk_sel[5])
			clk2 = 0;
		if (clk_sel[5])
			clk3 = 1;
		else if (clk_sel[9])
			clk3 = 0;
		if (clk_sel[9])
			clk4 = 1;
		else if (clk_sel[13])
			clk4 = 0;
		
		chan_cnt_res = clk2 & chan_cnt_res_l & chan_c0;
		clk4_chc0 = chan_c0 & clk4;
		clk4_chc0n = ~chan_c0 & clk4;
			
		if (clk4)
			chan_sel_i <= 1;
		else if (clk2)
			chan_sel_i <= 0;
			
		if (clk1)
			chan_cnt[0] <= chan_cnt[1] + 6'h1;
		else if (chan_cnt_res)
			chan_cnt[0] <= 0;
		if (clk3)
			chan_cnt[1] <= chan_cnt[0];
			
		if (clk1)
			chan_c0_l <= chan_c0;
		
		if (clk3)
		begin
			if (~chan_c0_l)
			begin
				chan_latch1[0] <= chan_cnt[1][5:1];
				chan_latch1[2] <= chan_latch1[1];
				chan_latch2 <= chan_latch1[3];
			end
			else
			begin
				chan_latch1[1] <= chan_latch1[0];
				chan_latch1[3] <= chan_latch1[2];
				chan_latch2 <= chan_cnt[1][5:1];
			end
		end
		
		if (chan_sel_i)
			chan_sel <= chan_latch2;
		else
			chan_sel <= dest_channel_l2;
		
		if (clk2)
			chan_active_cnt_l <= chan_active_cnt;
		if (clk4_chc0n)
			chan_cmp_l <= chan_sel;
		
		if (clk4_chc0)
			chan_cnt_res_l <= chan_active_match | reset_reg;
		
		if (clk4_chc0)
			chan_cmp_l2 <= chan_cmp_l;
		if (wave_irq_set_cond | ramp_irq_set_cond)
			chan_irq <= chan_cmp_l2;
		if (clk1)
			chan_irq_l <= chan_irq;
		
		
		// IO
		
		if (glob_reg_l2[7:6] != 2'h0 | reset_reg | glob_data_wr2)
			glob_data_wr1 <= 0;
		else if (glob_data_exec)
			glob_data_wr1 <= 1;
		
		if (reset_reg | (clk_sel[9] & voice_rw_pending))
			glob_data_wr2 <= 0;
		else if (~glob_data_exec & glob_data_wr1)
			glob_data_wr2 <= 1;
		
		if (~glob_data_wr2)
		begin
			dest_channel_l2 <= dest_channel;
			glob_reg_l2 <= glob_reg;
			glob_data_l2[7:0] <= glob_data_l1;
			if (glob_data_exec)
				glob_data_l2[15:8] <= IO16 ? DATA_i[15:8] : DATA_i[7:0];
		end
		
		w434 = clk3 & voice_read_pending[1] & ~voice_write_pending[1];
		
		if (glob_reg_l2[7:6] != 2'h2 | reset_reg | glob_data_rd2)
			glob_data_rd1 <= 0;
		else if (cpu_write3)
			glob_data_rd1 <= 1;
		
		if (reset_reg | w434)
			glob_data_rd2 <= 0;
		else if (~cpu_write3 & glob_data_rd1)
			glob_data_rd2 <= 1;
		
		if (reset_reg | (~w434 & ~glob_data_rd2))
			glob_data_rd3 <= 0;
		else if (glob_data_rd2)
			glob_data_rd3 <= 1;
		
		if (clk_sel[14] | reset_reg)
		begin
			voice_write_pending[0] <= 1'h0;
			voice_write_pending[1] <= 1'h0;
			voice_read_pending[0] <= 1'h0;
			voice_read_pending[1] <= 1'h0;
		end
		else if (clk_sel[1])
		begin
			voice_write_pending[1] <= voice_write_pending[0];
			voice_read_pending[1] <= voice_read_pending[0];
		end
		else
		begin
			voice_write_pending[0] <= glob_data_wr2;
			voice_read_pending[0] <= glob_data_rd2;
		end
		
		ram_strobe = clk_sel[3] | clk_sel[11];
		if (voice_write_pending[1] & clk3)
			ram_wr_strobe = 1'h1;
		else if (clk4)
			ram_wr_strobe = 1'h0;
		
		if (clk3)
			voice_irq_read_l <= 1'h0;
		
		if (ram_strobe & clk2 & voice_rw_pending)
		begin
			glob_data_bus <= 16'hffff; // precharge
		end
		else if (ram_wr_strobe)
		begin
			glob_data_bus <= glob_data_bus & ~glob_data_l2;
		end
		else if (w434)
		begin
			if (voice_reg_E)
				glob_data_bus[12:8] <= glob_data_bus[12:8] & chan_active_cnt;
			else if (voice_reg_F)
			begin
				glob_data_bus[15:8] <= glob_data_bus[15:8] & { ~wave_irq_set_l, ~ramp_irq_set_l, 1'h1, chan_irq };
				voice_irq_read_l <= 1'h1;
			end
			else if (voice_reg_0)
				glob_data_bus[15:8] <= glob_data_bus[15:8] & ram_right_bus[147:140];
			else if (voice_reg_1)
			begin
				glob_data_bus[15:1] <= glob_data_bus[15:1] & {
					ram_right_bus[94],
					ram_right_bus[90],
					ram_right_bus[86],
					ram_right_bus[82],
					ram_right_bus[78],
					ram_right_bus[74],
					ram_right_bus[70],
					ram_right_bus[66],
					ram_right_bus[62],
					ram_right_bus[58],
					ram_right_bus[56],
					ram_right_bus[54],
					ram_right_bus[52],
					ram_right_bus[50],
					ram_right_bus[48] };
			end
			else if (voice_reg_2)
			begin
				glob_data_bus[12:0] <= glob_data_bus[12:0] & {
					ram_right_bus[138],
					ram_right_bus[135],
					ram_right_bus[132],
					ram_right_bus[129],
					ram_right_bus[126],
					ram_right_bus[123],
					ram_right_bus[120],
					ram_right_bus[117],
					ram_right_bus[114],
					ram_right_bus[111],
					ram_right_bus[108],
					ram_right_bus[105],
					ram_right_bus[102] };
			end
			else if (voice_reg_3)
			begin
				glob_data_bus[15:5] <= glob_data_bus[15:5] & {
					ram_right_bus[100],
					ram_right_bus[96],
					ram_right_bus[92],
					ram_right_bus[88],
					ram_right_bus[84],
					ram_right_bus[80],
					ram_right_bus[76],
					ram_right_bus[72],
					ram_right_bus[68],
					ram_right_bus[64],
					ram_right_bus[60] };
			end
			else if (voice_reg_4)
			begin
				glob_data_bus[12:0] <= glob_data_bus[12:0] & {
					ram_right_bus[137],
					ram_right_bus[134],
					ram_right_bus[131],
					ram_right_bus[128],
					ram_right_bus[125],
					ram_right_bus[122],
					ram_right_bus[119],
					ram_right_bus[116],
					ram_right_bus[113],
					ram_right_bus[110],
					ram_right_bus[107],
					ram_right_bus[104],
					ram_right_bus[101] };
			end
			else if (voice_reg_5)
			begin
				glob_data_bus[15:5] <= glob_data_bus[15:5] & {
					ram_right_bus[99],
					ram_right_bus[95],
					ram_right_bus[91],
					ram_right_bus[87],
					ram_right_bus[83],
					ram_right_bus[79],
					ram_right_bus[75],
					ram_right_bus[71],
					ram_right_bus[67],
					ram_right_bus[63],
					ram_right_bus[59] };
			end
			else if (voice_reg_6)
			begin
				glob_data_bus[15:8] <= glob_data_bus[15:8] & {
					ram_right_bus[43],
					ram_right_bus[42],
					ram_right_bus[12],
					ram_right_bus[8],
					ram_right_bus[6],
					ram_right_bus[4],
					ram_right_bus[2],
					ram_right_bus[0] };
			end
			else if (voice_reg_7)
			begin
				glob_data_bus[15:8] <= glob_data_bus[15:8] & {
					ram_right_bus[32],
					ram_right_bus[29],
					ram_right_bus[26],
					ram_right_bus[23],
					ram_right_bus[20],
					ram_right_bus[17],
					ram_right_bus[14],
					ram_right_bus[10] };
			end
			else if (voice_reg_8)
			begin
				glob_data_bus[15:8] <= glob_data_bus[15:8] & {
					ram_right_bus[31],
					ram_right_bus[28],
					ram_right_bus[25],
					ram_right_bus[22],
					ram_right_bus[19],
					ram_right_bus[16],
					ram_right_bus[13],
					ram_right_bus[9] };
			end
			else if (voice_reg_9)
			begin
				glob_data_bus[15:4] <= glob_data_bus[15:4] & {
					ram_right_bus[33],
					ram_right_bus[30],
					ram_right_bus[27],
					ram_right_bus[24],
					ram_right_bus[21],
					ram_right_bus[18],
					ram_right_bus[15],
					ram_right_bus[11],
					ram_right_bus[7],
					ram_right_bus[5],
					ram_right_bus[3],
					ram_right_bus[1] };
			end
			else if (voice_reg_A)
			begin
				glob_data_bus[12:0] <= glob_data_bus[12:0] & {
					ram_right_bus[139],
					ram_right_bus[136],
					ram_right_bus[133],
					ram_right_bus[130],
					ram_right_bus[127],
					ram_right_bus[124],
					ram_right_bus[121],
					ram_right_bus[118],
					ram_right_bus[115],
					ram_right_bus[112],
					ram_right_bus[109],
					ram_right_bus[106],
					ram_right_bus[103] };
			end
			else if (voice_reg_B)
			begin
				glob_data_bus <= glob_data_bus & {
					ram_right_bus[100],
					ram_right_bus[97],
					ram_right_bus[93],
					ram_right_bus[89],
					ram_right_bus[85],
					ram_right_bus[81],
					ram_right_bus[77],
					ram_right_bus[73],
					ram_right_bus[69],
					ram_right_bus[65],
					ram_right_bus[61],
					ram_right_bus[57],
					ram_right_bus[55],
					ram_right_bus[53],
					ram_right_bus[51],
					ram_right_bus[49] };
			end
			else if (voice_reg_C)
			begin
				glob_data_bus[11:8] <= glob_data_bus[11:8] & {
					ram_right_bus[47],
					ram_right_bus[46],
					ram_right_bus[45],
					ram_right_bus[44] };
			end
			else if (voice_reg_D)
			begin
				glob_data_bus[15:8] <= glob_data_bus[15:8] & {
					ram_right_bus[41],
					ram_right_bus[40],
					ram_right_bus[39],
					ram_right_bus[38],
					ram_right_bus[37],
					ram_right_bus[36],
					ram_right_bus[35],
					ram_right_bus[34] };
			end
		end
		
		if (clk_sel[4])
			voice_reg_sel <= glob_reg_l2[5:0];
		
		
			
		//if (~chan_c0)
		//	voice_ram_update_pipeline = clk2;
		voice_ram_update_pipeline = chan_c0 & clk2;
		
		//if (voice_write_pending[1])
		//	w1816 = clk2;
		w1816 = voice_write_pending[1] & clk4;
		
		voice_ram_update = voice_ram_update_pipeline | w1816;
		
		//if (voice_rw_pending)
		//	w1848 = clk3;
		w1848 = voice_rw_pending & clk3;
		
		voice_ram_read = clk1 | w1848;
		
		if (voice_ram_read)
			ram_addr_l = chan_sel;
		
		if (voice_ram_read)
			ram_right_bus <= voice_ram[ram_addr_l];
		else if ((clk_sel[4] & ~clk1) | clk_sel[12])
			ram_right_bus <= ~148'h0;
		if (voice_ram_update)
			voice_ram[ram_addr_l] <= ram_left_bus;
		
		//if (voice_rw_pending)
		//	w1823 = clk3;
		w1823 = voice_rw_pending & clk3;
		//if (chan_c0)
		//	w1845 = clk1;
		w1845 = chan_c0 & clk1;
		
		w100 = clk4 & voice_write_pending[1];
		
		if (w100)
		begin
			if (voice_reg_E)
				chan_active_cnt <= ~glob_data_bus[12:8];
			else if (voice_reg_0)
				ram_input_latch[147:140] <= glob_data_bus[15:8];
			else if (voice_reg_1)
			begin
				ram_input_latch[94] <= glob_data_bus[15];
				ram_input_latch[90] <= glob_data_bus[14];
				ram_input_latch[86] <= glob_data_bus[13];
				ram_input_latch[82] <= glob_data_bus[12];
				ram_input_latch[78] <= glob_data_bus[11];
				ram_input_latch[74] <= glob_data_bus[10];
				ram_input_latch[70] <= glob_data_bus[9];
				ram_input_latch[66] <= glob_data_bus[8];
				ram_input_latch[62] <= glob_data_bus[7];
				ram_input_latch[58] <= glob_data_bus[6];
				ram_input_latch[56] <= glob_data_bus[5];
				ram_input_latch[54] <= glob_data_bus[4];
				ram_input_latch[52] <= glob_data_bus[3];
				ram_input_latch[50] <= glob_data_bus[2];
				ram_input_latch[48] <= glob_data_bus[1];
			end
			else if (voice_reg_2)
			begin
				ram_input_latch[138] <= glob_data_bus[12];
				ram_input_latch[135] <= glob_data_bus[11];
				ram_input_latch[132] <= glob_data_bus[10];
				ram_input_latch[129] <= glob_data_bus[9];
				ram_input_latch[126] <= glob_data_bus[8];
				ram_input_latch[123] <= glob_data_bus[7];
				ram_input_latch[120] <= glob_data_bus[6];
				ram_input_latch[117] <= glob_data_bus[5];
				ram_input_latch[114] <= glob_data_bus[4];
				ram_input_latch[111] <= glob_data_bus[3];
				ram_input_latch[108] <= glob_data_bus[2];
				ram_input_latch[105] <= glob_data_bus[1];
				ram_input_latch[102] <= glob_data_bus[0];
			end
			else if (voice_reg_3)
			begin
				ram_input_latch[100] <= glob_data_bus[15];
				ram_input_latch[96] <= glob_data_bus[14];
				ram_input_latch[92] <= glob_data_bus[13];
				ram_input_latch[88] <= glob_data_bus[12];
				ram_input_latch[84] <= glob_data_bus[11];
				ram_input_latch[80] <= glob_data_bus[10];
				ram_input_latch[76] <= glob_data_bus[9];
				ram_input_latch[72] <= glob_data_bus[8];
				ram_input_latch[68] <= glob_data_bus[7];
				ram_input_latch[64] <= glob_data_bus[6];
				ram_input_latch[60] <= glob_data_bus[5];
			end
			else if (voice_reg_4)
			begin
				ram_input_latch[137] <= glob_data_bus[12];
				ram_input_latch[134] <= glob_data_bus[11];
				ram_input_latch[131] <= glob_data_bus[10];
				ram_input_latch[128] <= glob_data_bus[9];
				ram_input_latch[125] <= glob_data_bus[8];
				ram_input_latch[122] <= glob_data_bus[7];
				ram_input_latch[119] <= glob_data_bus[6];
				ram_input_latch[116] <= glob_data_bus[5];
				ram_input_latch[113] <= glob_data_bus[4];
				ram_input_latch[110] <= glob_data_bus[3];
				ram_input_latch[107] <= glob_data_bus[2];
				ram_input_latch[104] <= glob_data_bus[1];
				ram_input_latch[101] <= glob_data_bus[0];
			end
			else if (voice_reg_5)
			begin
				ram_input_latch[99] <= glob_data_bus[15];
				ram_input_latch[95] <= glob_data_bus[14];
				ram_input_latch[91] <= glob_data_bus[13];
				ram_input_latch[87] <= glob_data_bus[12];
				ram_input_latch[83] <= glob_data_bus[11];
				ram_input_latch[79] <= glob_data_bus[10];
				ram_input_latch[75] <= glob_data_bus[9];
				ram_input_latch[71] <= glob_data_bus[8];
				ram_input_latch[67] <= glob_data_bus[7];
				ram_input_latch[63] <= glob_data_bus[6];
				ram_input_latch[59] <= glob_data_bus[5];
			end
			else if (voice_reg_6)
			begin
				ram_input_latch[43] <= glob_data_bus[15];
				ram_input_latch[42] <= glob_data_bus[14];
				ram_input_latch[12] <= glob_data_bus[13];
				ram_input_latch[8] <= glob_data_bus[12];
				ram_input_latch[6] <= glob_data_bus[11];
				ram_input_latch[4] <= glob_data_bus[10];
				ram_input_latch[2] <= glob_data_bus[9];
				ram_input_latch[0] <= glob_data_bus[8];
			end
			else if (voice_reg_7)
			begin
				ram_input_latch[32] <= glob_data_bus[15];
				ram_input_latch[29] <= glob_data_bus[14];
				ram_input_latch[26] <= glob_data_bus[13];
				ram_input_latch[23] <= glob_data_bus[12];
				ram_input_latch[20] <= glob_data_bus[11];
				ram_input_latch[17] <= glob_data_bus[10];
				ram_input_latch[14] <= glob_data_bus[9];
				ram_input_latch[10] <= glob_data_bus[8];
			end
			else if (voice_reg_8)
			begin
				ram_input_latch[31] <= glob_data_bus[15];
				ram_input_latch[28] <= glob_data_bus[14];
				ram_input_latch[25] <= glob_data_bus[13];
				ram_input_latch[22] <= glob_data_bus[12];
				ram_input_latch[19] <= glob_data_bus[11];
				ram_input_latch[16] <= glob_data_bus[10];
				ram_input_latch[13] <= glob_data_bus[9];
				ram_input_latch[9] <= glob_data_bus[8];
			end
			else if (voice_reg_9)
			begin
				ram_input_latch[33] <= glob_data_bus[15];
				ram_input_latch[30] <= glob_data_bus[14];
				ram_input_latch[27] <= glob_data_bus[13];
				ram_input_latch[24] <= glob_data_bus[12];
				ram_input_latch[21] <= glob_data_bus[11];
				ram_input_latch[18] <= glob_data_bus[10];
				ram_input_latch[15] <= glob_data_bus[9];
				ram_input_latch[11] <= glob_data_bus[8];
				ram_input_latch[7] <= glob_data_bus[7];
				ram_input_latch[5] <= glob_data_bus[6];
				ram_input_latch[3] <= glob_data_bus[5];
				ram_input_latch[1] <= glob_data_bus[4];
			end
			else if (voice_reg_A)
			begin
				ram_input_latch[139] <= glob_data_bus[12];
				ram_input_latch[136] <= glob_data_bus[11];
				ram_input_latch[133] <= glob_data_bus[10];
				ram_input_latch[130] <= glob_data_bus[9];
				ram_input_latch[127] <= glob_data_bus[8];
				ram_input_latch[124] <= glob_data_bus[7];
				ram_input_latch[121] <= glob_data_bus[6];
				ram_input_latch[118] <= glob_data_bus[5];
				ram_input_latch[115] <= glob_data_bus[4];
				ram_input_latch[112] <= glob_data_bus[3];
				ram_input_latch[109] <= glob_data_bus[2];
				ram_input_latch[106] <= glob_data_bus[1];
				ram_input_latch[103] <= glob_data_bus[0];
			end
			else if (voice_reg_B)
			begin
				ram_input_latch[100] <= glob_data_bus[15];
				ram_input_latch[97] <= glob_data_bus[14];
				ram_input_latch[93] <= glob_data_bus[13];
				ram_input_latch[89] <= glob_data_bus[12];
				ram_input_latch[85] <= glob_data_bus[11];
				ram_input_latch[81] <= glob_data_bus[10];
				ram_input_latch[77] <= glob_data_bus[9];
				ram_input_latch[73] <= glob_data_bus[8];
				ram_input_latch[69] <= glob_data_bus[7];
				ram_input_latch[65] <= glob_data_bus[6];
				ram_input_latch[61] <= glob_data_bus[5];
				ram_input_latch[57] <= glob_data_bus[4];
				ram_input_latch[55] <= glob_data_bus[3];
				ram_input_latch[53] <= glob_data_bus[2];
				ram_input_latch[51] <= glob_data_bus[1];
				ram_input_latch[49] <= glob_data_bus[0];
			end
			else if (voice_reg_C)
			begin
				ram_input_latch[47] <= glob_data_bus[11];
				ram_input_latch[46] <= glob_data_bus[10];
				ram_input_latch[45] <= glob_data_bus[9];
				ram_input_latch[44] <= glob_data_bus[8];
			end
			else if (voice_reg_D)
			begin
				ram_input_latch[41] <= glob_data_bus[15];
				ram_input_latch[40] <= glob_data_bus[14];
				ram_input_latch[39] <= glob_data_bus[13];
				ram_input_latch[38] <= glob_data_bus[12];
				ram_input_latch[37] <= glob_data_bus[11];
				ram_input_latch[36] <= glob_data_bus[10];
				ram_input_latch[35] <= glob_data_bus[9];
				ram_input_latch[34] <= glob_data_bus[8];
			end
		end
		else if (w1823 | w1845)
		begin
			ram_input_latch <= ~ram_right_bus; 
		end
		
		//if (~chan_c0)
		//	w1885 = clk1;
		w1885 = ~chan_c0 & clk1;
		
		if (w1885)
			ram_output_latch <= ram_right_bus;
		
		//if (chan_c0)
		//	w134 = clk2;
		w134 = chan_c0 & clk2;
		
		if (clk4)
		begin
			if (chan_c0)
			begin
				wave_is_stopped_l[0] <= wave_is_stopped;
				wave_is_stopped_l[2] <= wave_is_stopped_l[1];
				wave_is_stopped_l[4] <= wave_is_stopped_l[3];
				
				ramp_is_stopped_l[0] <= ramp_is_stopped;
				ramp_is_stopped_l[2] <= ramp_is_stopped_l[1];
				ramp_is_stopped_l[4] <= ramp_is_stopped_l[3];
			end
			else
			begin
				wave_is_stopped_l[1] <= wave_is_stopped_l[0];
				wave_is_stopped_l[3] <= wave_is_stopped_l[2];
				
				ramp_is_stopped_l[1] <= ramp_is_stopped_l[0];
				ramp_is_stopped_l[3] <= ramp_is_stopped_l[2];
			end
		end
		
		//if (chan_c0 & ~wave_is_stopped_l[4])
		//	w1863 = clk2;
		w1863 = (chan_c0 & ~wave_is_stopped_l[4]) & clk2;
		
		//if (chan_c0 & ~ramp_is_stopped_l[4])
		//	w1982 = clk2;
		w1982 = (chan_c0 & ~ramp_is_stopped_l[4]) & clk2;
		
		if (w134)
		begin
			ram_left_bus[147] <= wave_params_l2[2][2] & ~wave_irq_clear_l5;
			ram_left_bus[41] <= ramp_params_l2[2][2] & ~ramp_irq_clear_l5;
		end
		if (w1863)
		begin
			ram_left_bus[146] <= wave_params_l2[2][1];
			ram_left_bus[140] <= wave_params_l2[2][0];
			ram_left_bus[139] <= wave_next_l[5][28];
			ram_left_bus[136] <= wave_next_l[5][27];
			ram_left_bus[133] <= wave_next_l[5][26];
			ram_left_bus[130] <= wave_next_l[5][25];
			ram_left_bus[127] <= wave_next_l[5][24];
			ram_left_bus[124] <= wave_next_l[5][23];
			ram_left_bus[121] <= wave_next_l[5][22];
			ram_left_bus[118] <= wave_next_l[5][21];
			ram_left_bus[115] <= wave_next_l[5][20];
			ram_left_bus[112] <= wave_next_l[5][19];
			ram_left_bus[109] <= wave_next_l[5][18];
			ram_left_bus[106] <= wave_next_l[5][17];
			ram_left_bus[103] <= wave_next_l[5][16];
			ram_left_bus[100] <= wave_next_l[5][15];
			ram_left_bus[97] <= wave_next_l[5][14];
			ram_left_bus[93] <= wave_next_l[5][13];
			ram_left_bus[89] <= wave_next_l[5][12];
			ram_left_bus[85] <= wave_next_l[5][11];
			ram_left_bus[81] <= wave_next_l[5][10];
			ram_left_bus[77] <= wave_next_l[5][9];
			ram_left_bus[73] <= wave_next_l[5][8];
			ram_left_bus[69] <= wave_next_l[5][7];
			ram_left_bus[65] <= wave_next_l[5][6];
			ram_left_bus[61] <= wave_next_l[5][5];
			ram_left_bus[57] <= wave_next_l[5][4];
			ram_left_bus[55] <= wave_next_l[5][3];
			ram_left_bus[53] <= wave_next_l[5][2];
			ram_left_bus[51] <= wave_next_l[5][1];
			ram_left_bus[49] <= wave_next_l[5][0];
		end
		if (w1982)
		begin
			ram_left_bus[40] <= ramp_params_l2[2][1];
			ram_left_bus[34] <= ramp_params_l2[2][0];
			ram_left_bus[33] <= ramp_next_l[5][11];
			ram_left_bus[30] <= ramp_next_l[5][10];
			ram_left_bus[27] <= ramp_next_l[5][9];
			ram_left_bus[24] <= ramp_next_l[5][8];
			ram_left_bus[21] <= ramp_next_l[5][7];
			ram_left_bus[18] <= ramp_next_l[5][6];
			ram_left_bus[15] <= ramp_next_l[5][5];
			ram_left_bus[11] <= ramp_next_l[5][4];
			ram_left_bus[7] <= ramp_next_l[5][3];
			ram_left_bus[5] <= ramp_next_l[5][2];
			ram_left_bus[3] <= ramp_next_l[5][1];
			ram_left_bus[1] <= ramp_next_l[5][0];
		end
		
		if (w134)
			w1800 = ram_strobe;
		if (w100 | (w134 & ~w1863))
			w1801 = ram_strobe;
		if (w100 | (w134 & ~w1982))
			w1944 = ram_strobe;
		if (w1800 | w1801)
		begin
			ram_left_bus[145] <= ~ram_input_latch[145];
			ram_left_bus[144] <= ~ram_input_latch[144];
			ram_left_bus[143] <= ~ram_input_latch[143];
			ram_left_bus[142] <= ~ram_input_latch[142];
			ram_left_bus[141] <= ~ram_input_latch[141];
			
			ram_left_bus[138] <= ~ram_input_latch[138];
			ram_left_bus[137] <= ~ram_input_latch[137];
			ram_left_bus[135] <= ~ram_input_latch[135];
			ram_left_bus[134] <= ~ram_input_latch[134];
			ram_left_bus[132] <= ~ram_input_latch[132];
			
			ram_left_bus[131] <= ~ram_input_latch[131];
			ram_left_bus[129] <= ~ram_input_latch[129];
			ram_left_bus[128] <= ~ram_input_latch[128];
			ram_left_bus[126] <= ~ram_input_latch[126];
			ram_left_bus[125] <= ~ram_input_latch[125];
			
			ram_left_bus[123] <= ~ram_input_latch[123];
			ram_left_bus[122] <= ~ram_input_latch[122];
			ram_left_bus[120] <= ~ram_input_latch[120];
			ram_left_bus[119] <= ~ram_input_latch[119];
			ram_left_bus[117] <= ~ram_input_latch[117];
			ram_left_bus[116] <= ~ram_input_latch[116];
			
			ram_left_bus[114] <= ~ram_input_latch[114];
			ram_left_bus[113] <= ~ram_input_latch[113];
			ram_left_bus[111] <= ~ram_input_latch[111];
			ram_left_bus[110] <= ~ram_input_latch[110];
			ram_left_bus[108] <= ~ram_input_latch[108];
			
			ram_left_bus[107] <= ~ram_input_latch[107];
			ram_left_bus[105] <= ~ram_input_latch[105];
			ram_left_bus[104] <= ~ram_input_latch[104];
			ram_left_bus[102] <= ~ram_input_latch[102];
			ram_left_bus[101] <= ~ram_input_latch[101];
			
			ram_left_bus[99] <= ~ram_input_latch[99];
			ram_left_bus[98] <= ~ram_input_latch[98];
			ram_left_bus[96] <= ~ram_input_latch[96];
			ram_left_bus[95] <= ~ram_input_latch[95];
			ram_left_bus[94] <= ~ram_input_latch[94];
			ram_left_bus[92] <= ~ram_input_latch[92];
			
			ram_left_bus[91] <= ~ram_input_latch[91];
			ram_left_bus[90] <= ~ram_input_latch[90];
			ram_left_bus[88] <= ~ram_input_latch[88];
			ram_left_bus[87] <= ~ram_input_latch[87];
			ram_left_bus[86] <= ~ram_input_latch[86];
			ram_left_bus[84] <= ~ram_input_latch[84];
			
			ram_left_bus[83] <= ~ram_input_latch[83];
			ram_left_bus[82] <= ~ram_input_latch[82];
			ram_left_bus[80] <= ~ram_input_latch[80];
			ram_left_bus[79] <= ~ram_input_latch[79];
			ram_left_bus[78] <= ~ram_input_latch[78];
			ram_left_bus[76] <= ~ram_input_latch[76];
			
			ram_left_bus[75] <= ~ram_input_latch[75];
			ram_left_bus[74] <= ~ram_input_latch[74];
			ram_left_bus[72] <= ~ram_input_latch[72];
			ram_left_bus[71] <= ~ram_input_latch[71];
			ram_left_bus[70] <= ~ram_input_latch[70];
			ram_left_bus[68] <= ~ram_input_latch[68];
			
			ram_left_bus[67] <= ~ram_input_latch[67];
			ram_left_bus[66] <= ~ram_input_latch[66];
			ram_left_bus[64] <= ~ram_input_latch[64];
			ram_left_bus[63] <= ~ram_input_latch[63];
			ram_left_bus[62] <= ~ram_input_latch[62];
			ram_left_bus[60] <= ~ram_input_latch[60];
			
			ram_left_bus[59] <= ~ram_input_latch[59];
			ram_left_bus[58] <= ~ram_input_latch[58];
			ram_left_bus[56] <= ~ram_input_latch[56];
			ram_left_bus[54] <= ~ram_input_latch[54];
			ram_left_bus[52] <= ~ram_input_latch[52];
			
			ram_left_bus[50] <= ~ram_input_latch[50];
			ram_left_bus[48] <= ~ram_input_latch[48];
			ram_left_bus[47] <= ~ram_input_latch[47];
			ram_left_bus[46] <= ~ram_input_latch[46];
			ram_left_bus[45] <= ~ram_input_latch[45];
			ram_left_bus[44] <= ~ram_input_latch[44];
			
			ram_left_bus[43] <= ~ram_input_latch[43];
			ram_left_bus[42] <= ~ram_input_latch[42];
			ram_left_bus[39] <= ~ram_input_latch[39];
			ram_left_bus[38] <= ~ram_input_latch[38];
			ram_left_bus[37] <= ~ram_input_latch[37];
			ram_left_bus[36] <= ~ram_input_latch[36];
			
			ram_left_bus[35] <= ~ram_input_latch[35];
			ram_left_bus[32] <= ~ram_input_latch[32];
			ram_left_bus[31] <= ~ram_input_latch[31];
			ram_left_bus[29] <= ~ram_input_latch[29];
			ram_left_bus[28] <= ~ram_input_latch[28];
			
			ram_left_bus[26] <= ~ram_input_latch[26];
			ram_left_bus[25] <= ~ram_input_latch[25];
			ram_left_bus[23] <= ~ram_input_latch[23];
			ram_left_bus[22] <= ~ram_input_latch[22];
			ram_left_bus[20] <= ~ram_input_latch[20];
			
			ram_left_bus[19] <= ~ram_input_latch[19];
			ram_left_bus[17] <= ~ram_input_latch[17];
			ram_left_bus[16] <= ~ram_input_latch[16];
			ram_left_bus[14] <= ~ram_input_latch[14];
			ram_left_bus[13] <= ~ram_input_latch[13];
			ram_left_bus[12] <= ~ram_input_latch[12];
			
			ram_left_bus[10] <= ~ram_input_latch[10];
			ram_left_bus[9] <= ~ram_input_latch[9];
			ram_left_bus[8] <= ~ram_input_latch[8];
			ram_left_bus[6] <= ~ram_input_latch[6];
			ram_left_bus[4] <= ~ram_input_latch[4];
			
			ram_left_bus[2] <= ~ram_input_latch[2];
			ram_left_bus[0] <= ~ram_input_latch[0];

		end
		if (w1801)
		begin
			ram_left_bus[147] <= ~ram_input_latch[147];
			ram_left_bus[146] <= ~ram_input_latch[146];
			ram_left_bus[140] <= ~ram_input_latch[140];
			
			ram_left_bus[139] <= ~ram_input_latch[139];
			ram_left_bus[136] <= ~ram_input_latch[136];
			ram_left_bus[133] <= ~ram_input_latch[133];
			
			ram_left_bus[130] <= ~ram_input_latch[130];
			ram_left_bus[127] <= ~ram_input_latch[127];
			ram_left_bus[124] <= ~ram_input_latch[124];
			
			ram_left_bus[121] <= ~ram_input_latch[121];
			ram_left_bus[118] <= ~ram_input_latch[118];
			
			ram_left_bus[115] <= ~ram_input_latch[115];
			ram_left_bus[112] <= ~ram_input_latch[112];
			ram_left_bus[109] <= ~ram_input_latch[109];
			
			ram_left_bus[106] <= ~ram_input_latch[106];
			ram_left_bus[103] <= ~ram_input_latch[103];
			ram_left_bus[100] <= ~ram_input_latch[100];
			
			ram_left_bus[97] <= ~ram_input_latch[97];
			ram_left_bus[93] <= ~ram_input_latch[93];
			
			ram_left_bus[89] <= ~ram_input_latch[89];
			ram_left_bus[85] <= ~ram_input_latch[85];
			
			ram_left_bus[81] <= ~ram_input_latch[81];
			ram_left_bus[77] <= ~ram_input_latch[77];
			
			ram_left_bus[73] <= ~ram_input_latch[73];
			ram_left_bus[69] <= ~ram_input_latch[69];
			
			ram_left_bus[65] <= ~ram_input_latch[65];
			ram_left_bus[61] <= ~ram_input_latch[61];
			
			ram_left_bus[57] <= ~ram_input_latch[57];
			ram_left_bus[55] <= ~ram_input_latch[55];
			ram_left_bus[53] <= ~ram_input_latch[53];
			
			ram_left_bus[51] <= ~ram_input_latch[51];
			ram_left_bus[49] <= ~ram_input_latch[49];
		end
		
		if (w1944)
		begin
			ram_left_bus[41] <= ~ram_input_latch[41];
			ram_left_bus[40] <= ~ram_input_latch[40];
			
			ram_left_bus[34] <= ~ram_input_latch[34];
			ram_left_bus[33] <= ~ram_input_latch[33];
			ram_left_bus[30] <= ~ram_input_latch[30];
			
			ram_left_bus[27] <= ~ram_input_latch[27];
			ram_left_bus[24] <= ~ram_input_latch[24];
			ram_left_bus[21] <= ~ram_input_latch[21];
			
			ram_left_bus[18] <= ~ram_input_latch[18];
			ram_left_bus[15] <= ~ram_input_latch[15];
			
			ram_left_bus[11] <= ~ram_input_latch[11];
			ram_left_bus[7] <= ~ram_input_latch[7];
			ram_left_bus[5] <= ~ram_input_latch[5];
			
			ram_left_bus[3] <= ~ram_input_latch[3];
			ram_left_bus[1] <= ~ram_input_latch[1];
		end
		
		if (clk_sel[8] | clk_sel[16])
			ram_left_bus <= 148'h0;
		
		
		// wave
		
		if (clk3)
		begin
			wave_params_l1[0] <= { wave_params[7], wave_params[6], wave_params[5], wave_params[4], wave_params[3], wave_params[0] };
			wave_roll[0] <= ramp_params[2];
		end
		if (clk1)
		begin
			wave_params_l1[1] <= wave_params_l1[0];
			wave_roll[1] <= wave_roll[0];
		end
		if (clk3)
		begin
			wave_params_l1[2] <= wave_params_l1[1];
			wave_roll[2] <= wave_roll[1];
		end
		
		if (clk4_chc0)
			wave_params_l2[0] <= { wave_irq_pend_next, wave_status_next, wave_dir_next };
		if (clk3)
			wave_params_l2[1] <= wave_params_l2[0];
		if (clk4_chc0)
			wave_params_l2[2] <= wave_params_l2[1];
		
		if (clk4)
			w1811 <= chan_c0;
		if (clk2)
			w1856 <= w1811;
		
		if (clk3)
			w1813 = 1'h1;
		if (clk1)
			w1813 = 1'h0;
		
		if (clk1)
			w1853 = 1'h1;
		if (clk3)
			w1853 = 1'h0;
		
		wave_phase1 = w1813 & w1856;
		wave_phase2 = w1813 & ~w1856;
		wave_phase3 = w1853 & w1811;
		wave_phase4 = w1853 & ~w1811;
		
		if (clk2 | clk4)
		begin
			wave_inc2 <= 29'h0;
			wave_inc2[9] <= wave_phase4;
			
			wave_sel1 <= 1'h1;
			wave_sel2 <= 1'h0;
			wave_sel3 <= 1'h0;
			
			if (wave_phase4)
			begin
				wave_sel2 <= 1'h1;
				wave_mux_hi <= wave_cur;
			end
			
			if (wave_phase2)
			begin
				if (wave_params[6]) // dir
					wave_sel3 <= 1'h1;
				wave_mux_lo <= wave_inc;
				wave_mux_hi <= wave_cur;
			end
			
			if (wave_phase3)
			begin
				wave_sel3 <= 1'h1;
				if (wave_params[6]) // dir
					wave_mux_lo <= wave_start;
				else
				begin
					wave_sel1 <= 1'h0;
					wave_mux_lo <= wave_end;
				end
				wave_mux_hi <= wave_sum_mem;
			end
			
			if (wave_phase1)
			begin
				if (~wave_is_stopped)
				begin
					if (wave_end_cond)
					begin
						if (wave_params[3]) // loop
						begin
							if (wave_params[6] ^ wave_params[4])
								wave_mux_lo <= wave_end;
							else
							begin
								wave_mux_lo <= wave_start;
								wave_sel3 <= 1'h1;
							end
							wave_sel1 <= 1'h0;
							wave_mux_hi <= wave_sum_mem;
						end
						else
						begin
							wave_sel2 <= 1'h1;
							wave_mux_hi <= wave_cur;
						end
					end
					else
					begin
						if (wave_params[6]) // dir
							wave_sel3 <= 1'h1;
						wave_mux_lo <= wave_inc;
						wave_mux_hi <= wave_cur;
					end
				end
				else
				begin
					wave_sel2 <= 1'h1;
					wave_mux_hi <= wave_cur;
				end
			end
		end
		
		if (clk2)
			wave_of_check_l <= chan_c0;
		
		if (wave_of_check_l & clk3)
			wave_end_reach <= ~wave_sum[29];
		
		if (clk1 | clk3)
			wave_sum_mem <= wave_sum[28:0];
		
		if (clk1)
		begin
			wave_next_l[0] <= wave_sum[28:0];
			wave_next_l[2] <= wave_next_l[1];
			wave_next_l[4] <= wave_next_l[3];
		end
		if (clk3)
		begin
			wave_next_l[1] <= wave_next_l[0];
			wave_next_l[3] <= wave_next_l[2];
			wave_next_l[5] <= wave_next_l[4];
		end
		
		if (clk3)
		begin
			wave_addr_l[0] <= wave_sum[28:0];
			wave_addr_l[2] <= wave_addr_l[1];
		end
		if (clk1 & chan_c0)
			wave_addr_l[1] <= wave_addr_l[0];
		else if (clk2 & ~chan_c0)
			wave_addr_l[1] <= wave_cur;
		
		// ramp
		
		if (clk3)
		begin
			ramp_params_l1[0] <= { ramp_params[7], ramp_params[6], ramp_params[5], ramp_params[4], ramp_params[3], ramp_params[0] };
		end
		if (clk1)
		begin
			ramp_params_l1[1] <= ramp_params_l1[0];
		end
		if (clk3)
		begin
			ramp_params_l1[2] <= ramp_params_l1[1];
		end
		
		if (clk4_chc0)
			ramp_params_l2[0] <= { ramp_irq_pend_next, ramp_status_next, ramp_dir_next };
		if (clk3)
			ramp_params_l2[1] <= ramp_params_l2[0];
		if (clk4_chc0)
			ramp_params_l2[2] <= ramp_params_l2[1];
		
		if (clk4)
			w1935 <= chan_c0;
		if (clk2)
			w1987 <= w1935;
		
		if (clk3)
			w1989 = 1'h1;
		if (clk1)
			w1989 = 1'h0;
		
		if (clk1)
			w1932 = 1'h1;
		if (clk3)
			w1932 = 1'h0;
		
		ramp_phase1 = w1989 & w1987;
		ramp_phase2 = w1989 & ~w1987;
		ramp_phase3 = w1932 & w1935;
		ramp_phase4 = w1932 & ~w1935;
		
		if (clk2 | clk4)
		begin
			ramp_sel1 <= 1'h1;
			ramp_sel2 <= 1'h0;
			ramp_sel3 <= 1'h0;
			
			if (ramp_phase4)
			begin
				ramp_sel2 <= 1'h1;
				ramp_mux_hi <= ramp_cur;
			end
			
			if (ramp_phase2)
			begin
				if (ramp_params[6]) // dir
					ramp_sel3 <= 1'h1;
				ramp_mux_lo <= ramp_inc;
				ramp_mux_hi <= ramp_cur;
			end
			
			if (ramp_phase3)
			begin
				ramp_sel3 <= 1'h1;
				if (ramp_params[6]) // dir
					ramp_mux_lo <= ramp_start;
				else
				begin
					ramp_sel1 <= 1'h0;
					ramp_mux_lo <= ramp_end;
				end
				ramp_mux_hi <= ramp_sum_mem;
			end
			
			if (ramp_phase1)
			begin
				if (~ramp_is_stopped)
				begin
					if (ramp_end_cond)
					begin
						if (ramp_params[3]) // loop
						begin
							if (ramp_params[6] ^ ramp_params[4])
								ramp_mux_lo <= ramp_end;
							else
							begin
								ramp_mux_lo <= ramp_start;
								ramp_sel3 <= 1'h1;
							end
							ramp_sel1 <= 1'h0;
							ramp_mux_hi <= ramp_sum_mem;
						end
						else
						begin
							ramp_sel2 <= 1'h1;
							ramp_mux_hi <= ramp_cur;
						end
					end
					else
					begin
						if (ramp_params[6]) // dir
							ramp_sel3 <= 1'h1;
						ramp_mux_lo <= ramp_inc;
						ramp_mux_hi <= ramp_cur;
					end
				end
				else
				begin
					ramp_sel2 <= 1'h1;
					ramp_mux_hi <= ramp_cur;
				end
			end
		end
		
		if (clk2)
			ramp_of_check_l <= chan_c0;
		
		if (ramp_of_check_l & clk3)
			ramp_end_reach <= ~ramp_sum[12];
		
		if (clk1 | clk3)
			ramp_sum_mem <= ramp_sum[11:0];
		
		if (clk1)
		begin
			ramp_next_l[0] <= ramp_sum[11:0];
			ramp_next_l[2] <= ramp_next_l[1];
			ramp_next_l[4] <= ramp_next_l[3];
		end
		if (clk3)
		begin
			ramp_next_l[1] <= ramp_next_l[0];
			ramp_next_l[3] <= ramp_next_l[2];
			ramp_next_l[5] <= ramp_next_l[4];
		end
		
		if (clk3)
		begin
			ramp_addr_l[0] <= ramp_sum[11:0];
			ramp_addr_l[2] <= ramp_addr_l[1];
		end
		if (clk1 & chan_c0)
			ramp_addr_l[1] <= ramp_addr_l[0];
		else if (clk2 & ~chan_c0)
			ramp_addr_l[1] <= ramp_cur;
		
		if (reset_reg)
		begin
			ramp_cnt[0] <= 9'h0;
			ramp_cnt[1] <= 9'h0;
		end
		else if (~chan_cnt_res)
			ramp_cnt[0] <= ramp_cnt[1] + 9'h1;
		else
			ramp_cnt[1] <= ramp_cnt[0];
		
		if (w1885)
			ramp_ch_l <= ~chan_sel;
		
		if (clk3)
		begin
			case ({ram_output_latch[43], ram_output_latch[42]})
				2'h0: w368 <= 1'h1;
				2'h1: w368 <= ramp_ch_l[2:0] == ramp_cnt[1][2:0];
				2'h2: w368 <= ramp_ch_l == ramp_cnt[1][4:0] & ramp_cnt[1][5];
				2'h3: w368 <= ramp_ch_l == ramp_cnt[1][4:0] & ramp_cnt[1][8:5] == 4'hf;
				default: w368 <= 1'h1;
			endcase
		end
		
		// voice irq
		
		voice_irq_read_l2 = clk4 & voice_irq_read_l;
		
		wave_irq_set_cond = clk2 & ~chan_c0 & ~wave_irq_clear_l1 & ~wave_irq_set_l & ~ramp_irq_set_l & wave_irq_pend_next;
		
		if (reset_reg | voice_irq_read_l2)
			wave_irq_set <= 1'h0;
		else if (wave_irq_set_cond)
			wave_irq_set <= 1'h1;
		
		if (reset_reg | wave_irq_clear_l4)
			wave_irq_clear <= 1'h0;
		else if (voice_irq_read_l2 & wave_irq_set_l)
			wave_irq_clear <= 1'h1;
		
		if (clk3)
		begin
			wave_irq_set_l <= wave_irq_set;
			wave_irq_clear_l2 <= wave_irq_clear_l1;
			wave_irq_clear_l4 <= wave_irq_clear_l3;
		end
		
		if (clk4 & chan_c0)
		begin
			wave_irq_clear_l1 <= wave_irq_clear;
			wave_irq_clear_l3 <= wave_irq_clear_l2 & ~wave_irq_set & chan_irq_match;
			wave_irq_clear_l5 <= wave_irq_clear_l4;
		end
		
		
		ramp_irq_set_cond = clk2 & ~chan_c0 & ~ramp_irq_clear_l1 & ~wave_irq_set_l & ~ramp_irq_set_l & ramp_irq_pend_next;
		
		if (reset_reg | voice_irq_read_l2)
			ramp_irq_set <= 1'h0;
		else if (ramp_irq_set_cond)
			ramp_irq_set <= 1'h1;
		
		if (reset_reg | ramp_irq_clear_l4)
			ramp_irq_clear <= 1'h0;
		else if (voice_irq_read_l2 & ramp_irq_set_l)
			ramp_irq_clear <= 1'h1;
		
		if (clk3)
		begin
			ramp_irq_set_l <= ramp_irq_set;
			ramp_irq_clear_l2 <= ramp_irq_clear_l1;
			ramp_irq_clear_l4 <= ramp_irq_clear_l3;
		end
		
		if (clk4 & chan_c0)
		begin
			ramp_irq_clear_l1 <= ramp_irq_clear;
			ramp_irq_clear_l3 <= ramp_irq_clear_l2 & ~ramp_irq_set & chan_irq_match;
			ramp_irq_clear_l5 <= ramp_irq_clear_l4;
		end
		
		// glob registers
		if (cpu_write_5 & glob_addr_41)
		begin
			dram_dma_invert_msb <= glob_data_l2[15];
			dram_dma_data_size <= glob_data_l2[14];
			dram_dma_irq_en_l <= glob_data_l2[13];
			dram_dma_rate1 <= glob_data_l2[12];
			dram_dma_rate0 <= glob_data_l2[11];
			dram_dma_chan_width <= glob_data_l2[10];
			dram_dma_dir <= glob_data_l2[9];
			dram_dma_en_l <= glob_data_l2[8];
		end
		else
		begin
			dram_dma_irq_en <= dram_dma_irq_en_l;
			dram_dma_en <= dram_dma_en_l;
		end
		if (cpu_write_5 & glob_addr_45)
		begin
			timer_ctrl_b5_l <= glob_data_l2[13];
			timer_ctrl_b4_l <= glob_data_l2[12];
			timer2_irq_en_l <= glob_data_l2[11];
			timer1_irq_en_l <= glob_data_l2[10];
			timer_ctrl_b1_l <= glob_data_l2[9];
			timer_ctrl_b0_l <= glob_data_l2[8];
		end
		else
		begin
			timer_ctrl_b5 <= timer_ctrl_b5_l;
			timer_ctrl_b4 <= timer_ctrl_b4_l;
			timer2_irq_en <= timer2_irq_en_l;
			timer1_irq_en <= timer1_irq_en_l;
			timer_ctrl_b1 <= timer_ctrl_b1_l;
			timer_ctrl_b0 <= timer_ctrl_b0_l;
		end
		if (cpu_write_5 & glob_addr_46)
		begin
			timer1_reg_l <= glob_data_l2[15:8];
		end
		else
		begin
			timer1_reg <= timer1_reg_l;
		end
		if (cpu_write_5 & glob_addr_47)
		begin
			timer2_reg_l <= glob_data_l2[15:8];
		end
		else
		begin
			timer2_reg <= timer2_reg_l;
		end
		if (cpu_write_5 & glob_addr_4c)
		begin
			gf1_irq_enable_l <= glob_data_l2[10];
			dac_enable_l <= glob_data_l2[9];
			reset_reg_not_l <= glob_data_l2[8];
		end
		else
		begin
			gf1_irq_enable <= gf1_irq_enable_l;
			dac_enable <= dac_enable_l;
			reset_reg_not <= reset_reg_not_l;
		end
		
		if (reset_reg)
		begin
			dram_dma_invert_msb <= 1'h0;
			dram_dma_data_size <= 1'h0;
			dram_dma_irq_en_l <= 1'h0;
			dram_dma_irq_en <= 1'h0;
			dram_dma_rate1 <= 1'h0;
			dram_dma_rate0 <= 1'h0;
			dram_dma_chan_width <= 1'h0;
			dram_dma_dir <= 1'h0;
			dram_dma_en_l <= 1'h0;
			dram_dma_en <= 1'h0;
	
			timer_ctrl_b5_l <= 1'h0;
			timer_ctrl_b5 <= 1'h0;
			timer_ctrl_b4_l <= 1'h0;
			timer_ctrl_b4 <= 1'h0;
			timer2_irq_en_l <= 1'h0;
			timer2_irq_en <= 1'h0;
			timer1_irq_en_l <= 1'h0;
			timer1_irq_en <= 1'h0;
			timer_ctrl_b1_l <= 1'h0;
			timer_ctrl_b1 <= 1'h0;
			timer_ctrl_b0_l <= 1'h0;
			timer_ctrl_b0 <= 1'h0;
			
			gf1_irq_enable_l <= 1'h0;
			gf1_irq_enable <= 1'h0;
			dac_enable_l <= 1'h0;
			dac_enable <= 1'h0;
		end
		
		if (RESET)
		begin
			reset_reg_not_l <= 1'h0;
			reset_reg_not <= 1'h0;
		end
		
		if (((cpu_write_4 & IO16) | cpu_write_5) & glob_addr_43)
			dram_peek_address[19:4] <= glob_data_l2;
		if (cpu_write_5 & glob_addr_44)
			dram_peek_address[3:0] <= glob_data_l2[11:8];
		
		if (glob_addr_41)
		begin
			glob_read_mux = { 
				dram_dma_irq_pending,
				dram_dma_data_size,
				dram_dma_irq_en_l,
				dram_dma_rate1,
				dram_dma_rate0,
				dram_dma_chan_width,
				dram_dma_dir,
				dram_dma_en_l
				};
		end
		else if (glob_addr_45)
		begin
			glob_read_mux = { 
				2'h0,
				timer_ctrl_b5,
				timer_ctrl_b4,
				timer2_irq_en,
				timer1_irq_en,
				timer_ctrl_b1,
				timer_ctrl_b0
				};
		end
		else if (glob_addr_49)
		begin
			glob_read_mux = 8'h0; // stub
		end
		else if (glob_addr_4c)
		begin
			glob_read_mux = { 
				5'h0,
				gf1_irq_enable,
				dac_enable,
				reset_reg_not
				};
		end
		
		if (w434)
			glob_data_read <= glob_data_bus;
		
		
		// cpu data bus
		
		if (cpu_input)
			cpu_bus <= DATA_i;
		
		if (cpu_read)
		begin
			if (CS1) // GF1
			begin
				if (cpu_addr_1) // joystick
					cpu_bus[7:0] <= 8'h0; // stub
				if (glob_reg_l2[7:6] == 2'h2) // voice registers
				begin
					if (cpu_addr_4 | cpu_addr_5)
						cpu_bus[15:8] <= glob_data_read[15:8];
					if (cpu_addr_5 & ~IO16)
						cpu_bus[7:0] <= glob_data_read[15:8];
					if (cpu_addr_4 | (cpu_addr_5 & IO16))
						cpu_bus[7:0] <= glob_data_read[7:0];
				end
				if (glob_reg_l2[6] & (cpu_addr_4 | cpu_addr_5))
					cpu_bus <= glob_read_mux;
				if (cpu_addr_6) // irq status
					cpu_bus[7:0] <= { dma_tc_irq, ramp_irq, wave_irq, blaster_io_irq, timer2_irq, timer1_irq, midi_rx_irq, midi_tx_irq };
				if (cpu_addr_7) // mem peek
					cpu_bus[7:0] <= peek_reg;
				if (cpu_addr_8) // adlib status
				begin
					if (~timer_ctrl_b0)
						cpu_bus[7:0] <= { adlib_irq, timer1_expire, timer2_expire, blaster_io_dsp_req, blaster_io_dsp_reset_req, timer1_irq, timer2_irq, adlib_io_req };
					else
						cpu_bus[7:0] <= reg_2XA;
				end
				if (cpu_addr_9) // adlib data
					cpu_bus[7:0] <= adlib_data;
				if (cpu_addr_A) // adlib reg
					cpu_bus[7:0] <= adlib_reg;
				if (cpu_addr_C) // blaster dsp
					cpu_bus[7:0] <= reg_2XC;
				if (cpu_addr_E) // blaster status
					cpu_bus[7:0] <= reg_2XE;
			end
			if (CS2) // MIDI
			begin
				if (cpu_addr_0)
					cpu_bus[7:0] <= 8'h0; // stub
				if (cpu_addr_1)
					cpu_bus[7:0] <= 8'h0; // stub
			end
		end
		
		if (cpu_write)
		begin
			if (CS1)
			begin
				if (cpu_addr_2)
					dest_channel <= DATA_i[4:0];
				if (cpu_addr_3)
					glob_reg <= DATA_i[7:0];
				if (cpu_addr_4 | (cpu_addr_5 & IO16))
					glob_data_l1 <= DATA_i[7:0];
				if (cpu_addr_A)
					reg_2XA <= DATA_i[7:0];
				if (cpu_addr_C | cpu_addr_D)
					reg_2XC <= DATA_i[7:0];
				if (cpu_addr_E)
					reg_2XE <= DATA_i[7:0];
			end
			if (CS2)
			begin
			end
		end
		
		if (cpu_write & CS1 & cpu_addr_8)
		begin
			adlib_reg <= DATA_i[7:0];
			adlib_reg_sel_4[0] <= DATA_i[7:0] == 8'h4;
		end
		else
			adlib_reg_sel_4[1] <= adlib_reg_sel_4[0];
		
		if (cpu_write & CS1 & cpu_addr_9 & (~adlib_reg_sel_4[1] | timer_ctrl_b0))
		begin
			adlib_data <= DATA_i[7:0];
			adlib_io_req_l <= timer_ctrl_b1;
		end
		else
			adlib_io_req <= adlib_io_req_l;
		
		if ((cpu_read & CS1 & cpu_addr_9) | reset_reg)
		begin
			adlib_io_req_l <= 1'h0;
			adlib_io_req <= 1'h0;
		end
		
		if (cpu_write & CS1 & cpu_addr_9 & adlib_reg_sel_4[1] & ~timer_ctrl_b0)
		begin
			timer_irq_res_l <= DATA_i[7];
			timer1_mask_l <= DATA_i[7] ? timer1_mask : DATA_i[6];
			timer2_mask_l <= DATA_i[7] ? timer2_mask : DATA_i[5];
			timer2_start_l <= DATA_i[7] ? timer2_start : DATA_i[1];
			timer1_start_l <= DATA_i[7] ? timer1_start : DATA_i[0];
		end
		else
		begin
			timer_irq_res <= timer_irq_res_l;
			timer1_mask <= timer1_mask_l;
			timer2_mask <= timer2_mask_l;
			timer2_start <= timer2_start_l;
			timer1_start <= timer1_start_l;
		end
		
		if (reset_reg)
		begin
			timer1_mask_l <= 1'h0;
			timer1_mask <= 1'h0;
			timer2_mask_l <= 1'h0;
			timer2_mask <= 1'h0;
			timer2_start_l <= 1'h0;
			timer2_start <= 1'h0;
			timer1_start_l <= 1'h0;
			timer1_start <= 1'h0;
		end
		
		if (~adlib_irq)
		begin
			timer_irq_res_l <= 1'h0;
			timer_irq_res <= 1'h0;
		end
		
		if (cpu_read & CS1 & cpu_addr_C)
			blaster_io_dsp_req_l <= timer_ctrl_b5;
		else
			blaster_io_dsp_req <= blaster_io_dsp_req_l;
		
		if (cpu_read & CS1 & cpu_addr_6)
			blaster_io_dsp_reset_req_l <= timer_ctrl_b5;
		else
			blaster_io_dsp_reset_req <= blaster_io_dsp_reset_req_l;
		
		if (reset_reg | ~timer_ctrl_b5)
		begin
			blaster_io_dsp_req_l <= 1'h0;
			blaster_io_dsp_req <= 1'h0;
			blaster_io_dsp_reset_req_l <= 1'h0;
			blaster_io_dsp_reset_req <= 1'h0;
		end
		
		// midi & adlib timer clk div
		
		if (~CLK)
		begin
			clk_div_cnt[0] = clk_div_cnt[1] + 4'h1;
			clk_div_reset[1] = clk_div_reset[0];
		end
		else
		begin
			clk_div_cnt[1] = clk_div_cnt[0];
			clk_div_reset[0] = clk_div_cnt[1][3] & clk_div_cnt[1][1];
		end
		
		if (RESET | clk_div_reset[1])
		begin
			clk_div_cnt[0] = 4'h0;
			clk_div_cnt[1] = 4'h0;
		end
		if (RESET)
		begin
			clk_div_reset[0] = 1'h0;
			clk_div_reset[1] = 1'h0;
		end
		
		if (~clk_div_cnt[1][2])
		begin
			clk_div_cnt2[0] = clk_div_cnt2[1] + 7'h1;
			clk_div_reset2[1] = clk_div_reset2[0];
		end
		else
		begin
			clk_div_cnt2[1] = clk_div_cnt2[0];
			clk_div_reset2[0] = clk_div_cnt2[1][6] & clk_div_cnt2[1][3] & clk_div_cnt2[1][2] & clk_div_cnt2[1][1];
		end
		
		if (reset_reg | clk_div_reset2[1])
		begin
			clk_div_cnt2[0] = 7'h0;
			clk_div_cnt2[1] = 7'h0;
		end
		if (reset_reg)
		begin
			clk_div_reset2[0] = 1'h0;
			clk_div_reset2[1] = 1'h0;
		end
		
		// ~12.5khz
		clk_div_timer = ~((~clk_div_cnt2[1][6] & clk_div_cnt[1][2]) | (~clk_div_cnt2[1][6] & ~timer_ctrl_b4) | (clk_div_cnt[1][2] & timer_ctrl_b4));
		
		if (~clk_div_timer)
			clk_div_cnt3[0] = clk_div_cnt3[1] + 2'h1;
		else
			clk_div_cnt3[1] = clk_div_cnt3[0];
		if (reset_reg)
		begin
			clk_div_cnt3[0] = 1'h0;
			clk_div_cnt3[1] = 1'h0;
		end
		
		clk_div_timer2 = clk_div_cnt3[1][1];
		
		if (~clk_div_timer)
		begin
			if (timer1_start & ~timer1_of)
				timer1_cnt[0] <= timer1_cnt_add[7:0];
			else
				timer1_cnt[0] <= timer1_reg;
			
			timer1_irq_l <= timer1_irq | (timer1_start & timer1_of);
			timer1_expire_l <= timer1_expire | (~timer1_mask & timer1_start & timer1_of);
		end
		else
		begin
			timer1_cnt[1] <= timer1_cnt[0];
			timer1_irq <= timer1_irq_l;
			timer1_expire <= timer1_expire_l;
		end
		
		if (~timer1_irq_en)
		begin
			timer1_irq_l <= 1'h0;
			timer1_irq <= 1'h0;
		end
		
		if (~clk_div_timer2)
		begin
			if (timer2_start & ~timer2_of)
				timer2_cnt[0] <= timer2_cnt_add[7:0];
			else
				timer2_cnt[0] <= timer2_reg;
			
			timer2_irq_l <= timer2_irq | (timer2_start & timer2_of);
			timer2_expire_l <= timer2_expire | (~timer2_mask & timer2_start & timer2_of);
		end
		else
		begin
			timer2_cnt[1] <= timer2_cnt[0];
			timer2_irq <= timer2_irq_l;
			timer2_expire <= timer2_expire_l;
		end
		
		if (~timer2_irq_en)
		begin
			timer2_irq_l <= 1'h0;
			timer2_irq <= 1'h0;
		end
		
		if (reset_reg | timer_irq_res)
		begin
			timer1_expire_l <= 1'h0;
			timer1_expire <= 1'h0;
			timer2_expire_l <= 1'h0;
			timer2_expire <= 1'h0;
		end
		
		// dma
		
		if (clk_sel[1] & chan_c0)
			dram_refresh_slot <= 1'h1;
		else if (clk_sel[9])
			dram_refresh_slot <= 1'h0;
		
		if ((IOR | IOW) & DACK1 & dram_dma_en)
			dram_dma_tc_l <= DMA_TC;
		else
			dram_dma_tc <= dram_dma_tc_l;
		
		if (reset_reg | dram_dma_irq_ack)
		begin
			dram_dma_tc_l <= 1'h0;
			dram_dma_tc <= 1'h0;
		end
		
		if (dram_dma_tc)
			dram_dma_irq_pending <= 1'h1;
		else if (~dram_dma_tc & dram_dma_irq_ack)
			dram_dma_irq_pending <= 1'h0;
		
		cls9_dma_write = clk_sel[9] & ~dram_dma_dir & dram_dma_access;
		cls9_dma_read = clk_sel[9] & dram_dma_dir & dram_dma_access;
		
		if (cls9_dma_read)
			dram_dma_req_rd <= 1'h1;
		else if (DACK1 | ~dram_dma_en)
			dram_dma_req_rd <= 1'h0;
		
		if (~dram_dma_en | cls9_dma_write)
			dram_dma_req_wr1 <= 1'h1;
		else if (dram_dma_req_wr2)
			dram_dma_req_wr1 <= 1'h0;
		
		if (dram_dma_tick & ~dram_dma_dir & dram_dma_req_wr1)
			dram_dma_req_wr2 <= 1'h1;
		else if (DACK1 | ~dram_dma_en)
			dram_dma_req_wr2 <= 1'h0;
		
		if (dram_refresh_slot)
			dram_dma_clk_cnt[0] <= dram_dma_clk_cnt[1] + 3'h1;
		else
			dram_dma_clk_cnt[1] <= dram_dma_clk_cnt[0];
		
		if (reset_reg)
		begin
			dram_dma_clk_cnt[0] <= 0;
			dram_dma_clk_cnt[1] <= 0;
		end
		
		
		if (DACK1 & IOW)
			dram_dma_in_data <= DATA_i;
		
		if (~dram_dma_write_state1)
			dram_dma_hi_lo_byte_sel[0] <= ~dram_dma_hi_lo_byte_sel[1];
		else
			dram_dma_hi_lo_byte_sel[1] <= dram_dma_hi_lo_byte_sel[0];
		if (~dram_dma_en)
		begin
			dram_dma_hi_lo_byte_sel[0] <= 1'h0;
			dram_dma_hi_lo_byte_sel[1] <= 1'h0;
		end
		
		if (IOW & DACK1)
			dram_dma_write_state1 <= 1'h1;
		else if (dram_dma_dir | ~dram_dma_en | dram_dma_write_state2)
			dram_dma_write_state1 <= 1'h0;
		
		if (dram_dma_write_state1 & ~IOW)
			dram_dma_write_state2 <= 1'h1;
		else if (cls9_dma_write | ~dram_dma_en)
			dram_dma_write_state2 <= 1'h0;
		
		if (IOR & DACK1 & dram_dma_en & dram_dma_dir)
			cpu_bus <= dram_dma_rd_latch;
		
		if (IOR & DACK1)
			dram_dma_read_state1 <= 1'h1;
		else if (~dram_dma_en | dram_dma_read_state2)
			dram_dma_read_state1 <= 1'h0;
		
		if (dram_dma_en | (IOR & ~dram_dma_read_state1))
			dram_dma_read_state2 <= 1'h1;
		else if (cls9_dma_read)
			dram_dma_read_state2 <= 1'h0;
		
		if (dram_dma_read_state2 & dram_dma_dir & dram_dma_tick)
			dram_dma_read_state3 <= 1'h1;
		else if (cls9_dma_read | ~dram_dma_en)
			dram_dma_read_state3 <= 1'h0;
		
		
		if (((cpu_write_4 & IO16) | cpu_write_5) & glob_addr_42)
		begin
			dram_dma_address[0] <= { glob_data_l2, 4'h0 };
			dram_dma_address[1] <= { glob_data_l2, 4'h0 };
		end
		else if (~cls9_dma_read | dram_dma_write_state2)
			dram_dma_address[0] <= dram_dma_address[1] + 20'h1;
		else
			dram_dma_address[1] <= dram_dma_address[0];
		
		
		if (~clk_sel[1])
			dram_io_wr_dma_l <= dram_dma_write_state2;
		else
			dram_io_wr_dma <= dram_io_wr_dma_l;
		
		if (~clk_sel[1])
			dram_io_rd_dma_l <= dram_dma_read_state3;
		else
			dram_io_rd_dma <= dram_io_rd_dma_l;
		
		if (~clk_sel[1])
			dram_io_pp_l <= dram_cpu_pp;
		else
			dram_io_pp <= dram_io_pp_l;
		
		if (dram_refresh_slot | clk_sel[14] | reset_reg)
		begin
			dram_io_wr_dma_l <= 1'h0;
			dram_io_wr_dma <= 1'h0;
			dram_io_rd_dma_l <= 1'h0;
			dram_io_rd_dma <= 1'h0;
			dram_io_pp_l <= 1'h0;
			dram_io_pp <= 1'h0;
		end
		
		if (reset_reg | ~dram_cpu_pp)
			dram_pp_state <= 1'h0;
		else if (clk_sel[9] & dram_pp_enable)
			dram_pp_state <= 1'h1;
		
		if (dram_cpu_pp & ~dram_pp_state)
			peek_reg <= dram_bus[7:0];
		
		if (clk3)
			dram_voice_access_16 <= wave_params[2];
		
		if (dram_cpu_access_16 & clk_sel[1])
			dram_access_16 <= 1'h1;
		if (~dram_cpu_access_16 & clk_sel[1])
			dram_access_16 <= 1'h0;
		if (dram_voice_access_16 & clk_sel[9])
			dram_access_16 <= 1'h1;
		if (~dram_voice_access_16 & clk_sel[9])
			dram_access_16 <= 1'h0;
		
		if (clk_sel[2] | clk_sel[10])
			dram_in_16 <= dram_access_16;
		
		if (clk_sel[2] & dram_io_dir)
			DRAM_WE <= 1'h1;
		if (clk_sel[9])
			DRAM_WE <= 1'h0;
		
		if (clk_sel[6] | clk_sel[14])
			dram_subaddr <= 1'h1;
		if (clk_sel[9] | clk_sel[1])
			dram_subaddr <= 1'h0;
		
		if (clk_sel[1])
			dram_addr_cw_sel <= 1'h1;
		else if (clk_sel[9])
			dram_addr_cw_sel <= 1'h0;
		
		if (clk_sel[8] & clk_sel[9])
			dram_hi_sel <= 1'h1;
		if (clk_sel[11])
			dram_hi_sel <= 1'h0;
		if (clk_sel[16] & clk_sel[1])
			dram_hi_sel <= 1'h1;
		if (clk_sel[3])
			dram_hi_sel <= 1'h0;
		
		if (~dram_io_dir)
		begin
			dram_bus[15:8] <= dram_in[15:8];
			if (dram_in_16)
				dram_bus[7:0] <= dram_in[7:0];
			else
				dram_bus[7:0] <= dram_in[15:8];
		end
		if (dram_io_dir & dram_pp_enable & dram_cpu_poke)
			dram_bus[7:0] <= cpu_bus[7:0];
		if (dram_io_dir & dram_dma_access & ~dram_dma_dir)
			dram_bus <= dram_dma_in_data_sign;
		if (dram_dma_read_state3)
			dram_dma_rd_latch <= dram_bus;
		
		if (clk_sel[10])
			DRAM_RAS <= 1'h1;
		if (clk_sel[16])
			DRAM_RAS <= 1'h0;
		if (clk_sel[2] & cpu_dram_access)
			DRAM_RAS <= 1'h1;
		if (clk_sel[5] & dram_refresh_slot)
			DRAM_RAS <= 1'h1;
		if (clk_sel[7] & clk_sel[8])
			DRAM_RAS <= 1'h0;
		
		if (clk_sel[11] & clk_sel[12])
			dram_cas <= 1'h1;
		if (clk_sel[13] & clk_sel[14] & dram_voice_access_16)
			dram_cas <= 1'h0;
		else if (clk_sel[15])
			dram_cas <= 1'h1;
		if (clk_sel[1])
			dram_cas <= 1'h0;
		
		if (clk_sel[4] & (cpu_dram_access | dram_refresh_slot))
			dram_cas <= 1'h1;
		if (clk_sel[5] & clk_sel[6] & dram_cpu_access_16)
			dram_cas <= 1'h0;
		else if (clk_sel[7])
			dram_cas <= 1'h1;
		if (clk_sel[9])
			dram_cas <= 1'h0;
		
		if (clk_sel[2] & chan_c0)
			dram_cas_refresh <= 1'h1;
		if (clk_sel[9])
			dram_cas_refresh <= 1'h0;
		
		if (clk_sel[1] | clk_sel[9])
			dram_latch_hi <= 1'h1;
		if (clk_sel[6] | clk_sel[14])
			dram_latch_hi <= 1'h0;
			
		if (clk_sel[6] | clk_sel[14])
			dram_latch_lo <= 1'h1;
		if (clk_sel[4] | clk_sel[12])
			dram_latch_lo <= 1'h0;
		
		if (dram_latch_hi)
			dram_in[15:8] <= DRAM_DATA_i;
		if (dram_latch_lo)
			dram_in[7:0] <= DRAM_DATA_i;
		if (~dram_in_16)
			dram_in[7:0] <= 8'h0;
		
		// synth
		
		if (clk1)
			mul_val_a_l <= dram_in;
		if (clk3)
			mul_val_a <= mul_val_a_l;
		else if (clk1)
			mul_val_a <= mul_interp;
		
		
		w2056 = clk2 & chan_c0_l;
		w33 = ~chan_c0_l & clk2;
		w2036 = chan_c0_l & clk1;
		w2035 = ~chan_c0_l & clk1;
		//if (~chan_c0_l)
		//	w2415 = clk4;
		w2415 = ~chan_c0_l & clk4;
		//if (chan_c0_l)
		//	w2416 = clk4;
		w2416 = chan_c0_l & clk4;
		w1924 = ~chan_c0_l & clk3;
		w1923 = chan_c0_l & clk3;
		
		if (w1924)
		begin
			atten_l[0] <= ramp_cur;
			pan_l[0] <= ram_output_latch[47:44];
		end
		if (w1923)
		begin
			atten_l[1] <= atten_l[0];
			pan_l[1] <= pan_l[0];
		end
			
		
		if (w33)
			interp_in <= wave_cur[8:1];
		if (w2036)
			interp_in_mul <= 9'h100 - { 1'h0, interp_in };
		if (w2035)
			interp_in_mul <= { 1'h0, interp_in };

		if (clk3)
			mul_val_b <= interp_in_mul;
		else if (clk1)
			mul_val_b <= { 1'h1, ~atten[7:0] };
		
		mul_result <= mul_val_a_s * mul_val_b_s;
		
		if (w2415)
			mul_mem[0] <= mul_result[23:8];
		if (w2416)
			mul_mem[1] <= mul_result[23:8];
		
		if (w2036)
			mul_interp <= mul_mem[0] + mul_mem[1];
		
		if (w2415)
			w2048 <= 1'h1;
		else if (w2416)
			w2048 <= 1'h0;
		
		case (pan)
			4'h0: pan_atten = 9'h1ff;
			4'h1: pan_atten = 9'h01;
			4'h2: pan_atten = 9'h03;
			4'h3: pan_atten = 9'h05;
			4'h4: pan_atten = 9'h07;
			4'h5: pan_atten = 9'h09;
			4'h6: pan_atten = 9'h0b;
			4'h7: pan_atten = 9'h0d;
			4'h8: pan_atten = 9'h0f;
			4'h9: pan_atten = 9'h14;
			4'ha: pan_atten = 9'h18;
			4'hb: pan_atten = 9'h1d;
			4'hc: pan_atten = 9'h25;
			4'hd: pan_atten = 9'h2d;
			4'he: pan_atten = 9'h3f;
			default: pan_atten = 9'h0;
		endcase
		
		if (clk2)
		begin
			val_shifted <= mul_result[24:9] >>> atten[11:8];
		end
		
		if (w2416)
			accum_l[0] <= accum_sum;
		if (w2415)
			accum_l[1] <= accum_sum;
		
		if (w33)
			w2360 <= 1'h0;
		else if (chan_cnt_res)
			w2360 <= 1'h1;
		
		if (w2416)
		begin
			w2360_l[0] <= w2360;
			w2360_l[2] <= w2360_l[1];
		end
		else if (w2056)
		begin
			w2360_l[1] <= w2360_l[0];
			w2360_l[3] <= w2360_l[2];
		end
		
		if (w2360_l[3])
			accum_add <= 21'h0;
		else if (w2056)
			accum_add <= accum_l[0];
		else if (w33)
			accum_add <= accum_l[1];
		
		if (w2360_l[1] & w2415)
			accum_mem <= accum_clip;
		
		if (CLK)
		begin
			dac_clk1_l[0] = clk1;
			dac_clk2_l[0] = clk2;
			dac_clk3_l[0] = clk3;
			dac_clk4_l[0] = clk4;
		end
		else
		begin
			dac_clk1_l[1] = dac_clk1_l[0];
			dac_clk2_l[1] = dac_clk2_l[0];
			dac_clk3_l[1] = dac_clk3_l[0];
			dac_clk4_l[1] = dac_clk4_l[0];
		end
		
		w2003 = (clk1 & dac_clk1_l[1])
			| (clk2 & dac_clk2_l[1])
			| (clk3 & dac_clk3_l[1])
			| (clk4 & dac_clk4_l[1]);
		
		if (clk1)
			w262 = 1'h1;
		else if (chan_cnt_res)
			w262 = 1'h0;
		
		if (clk3)
		begin
			w262_l[0] <= w262;
			w262_l[2] <= w262_l[1];
		end
		else
			w262_l[1] <= w262_l[0];
		
		
		if (w2003)
			dac_counter[0] <= dac_counter[1] + 6'h1;
		else
			dac_counter[1] <= dac_counter[0];
		
		if (w2007)
		begin
			dac_counter[0] <= 0;
			dac_counter[1] <= 0;
		end
		
		if (w2003)
			w2008[0] <= dac_counter[1][5] & dac_counter[1][4] & dac_counter[1][2];
		else
			w2008[1] <= w2008[0];
		
		if (~dac_enable | w2008[1])
			w2007 <= 1'h1;
		else if (~w262_l[2])
			w2007 <= 1'h0;
		
		if (w2038)
			accum_shifter[0] <= w1998 ? (w2059 ? accum_mem : accum_clip) : { accum_shifter[1][14:0], 1'h0 };
		else
			accum_shifter[1] <= accum_shifter[0];
		
		DAC_CLK <= w2003;
		
		if (w2021 & w2003)
			DAC_LR <= 1'h1;
		if (~dac_enable | (w2020 & w2003))
			DAC_LR <= 1'h0;
		
		if ((w2020 | w2021) & w2003)
			DAC_LR2 <= 1'h1;
		if (~dac_enable | ((w2055 | w2040) & w2003))
			DAC_LR2 <= 1'h0;
		
	end

	initial begin
		chan_active_cnt = 0;
		reset_reg_not = 0;
		
		voice_ram[0] <= 148'h0;
		
		clk_sel[0] = 0;
		clk_sel[1] = 0;
		clk_sel[2] = 0;
		clk_sel[3] = 0;
		clk_sel[4] = 0;
		clk_sel[5] = 0;
		clk_sel[6] = 0;
		clk_sel[7] = 0;
		clk_sel[8] = 0;
		clk_sel[9] = 0;
		clk_sel[10] = 0;
		clk_sel[11] = 0;
		clk_sel[12] = 0;
		clk_sel[13] = 0;
		clk_sel[14] = 0;
		clk_sel[15] = 0;
		clk_sel[16] = 0;
		
		chan_cnt_res_l = 0;
		
		chan_cnt[0] = 0;
		chan_cnt[1] = 0;
	end

endmodule
