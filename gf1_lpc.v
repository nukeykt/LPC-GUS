module gf1_lpc
	(
	input CLK, // LPC Clock
	input reset,
	input LPC_FRM,
	inout [3:0] LPC_DATA,
	inout SERIRQ,
	output reg LPC_DREQ,

	output [15:0] audio_l,
	output [15:0] audio_r
	);
	
	reg [16:0] gf1_clk;
	
	wire gf1_clk2 = gf1_clk[16];
	
	always @(posedge CLK)
	begin
		gf1_clk <= gf1_clk + 38843;
	end

	reg IOW;
	reg IOR;
	reg IO16;
	reg DACK1;
	reg [7:0] dram[0:262143];
	
	reg o_RAS;
	reg o_CAS;
	
	wire RAS;
	wire CAS;
	
	wire CASA = CAS & RAS;
	
	reg [15:0] dac_shifter;
	reg [15:0] dac_left;
	reg [15:0] dac_right;
	reg o_dac_clk;
	wire dac_clk;
	wire dac_data;
	wire dac_lr;
	reg o_dac_lr;
	
	assign audio_l = dac_left;
	assign audio_r = dac_right;
	
	wire [8:0] DADDR;
	wire [7:0] DRAM_o;
	reg [7:0] DRAM_i;
	reg [17:0] DADDR_l;
	reg DWE;
	wire DRAM_WE;
	
	
	reg [7:0] lpc_cnt;
	reg lpc_state;
	reg [3:0] lpc_out;
	reg io_write;
	reg io_read;
	reg io_dmawrite;
	reg io_dmaread;
	reg [7:0] dma_req;
	wire DREQ;
	reg o_DREQ;
	
	reg sr_state;
	reg sr_value;
	reg sr_clear;
	
	reg irq_sleep;
	reg [7:0] irq_state;
	wire IRQ1;
	wire IRQ2;
	wire IRQ = IRQ1 | IRQ2;
	reg o_IRQ;
	
	wire [15:0] gusbase = 16'h240;
	
	reg [15:0] io_address;
	reg [15:0] wr_data;
	wire [15:0] rd_data_gf1;
	reg [15:0] rd_data;
	reg [2:0] io_dma_chan;
	reg [1:0] io_dma_size;
	reg io_dma_tc;
	wire gf1_wait;
	
	wire isgf1addr = (io_address[15:4] == gusbase[15:4]
		& (io_address[3:0] == 4'h6 | io_address[3:0] == 4'h8 | io_address[3:0] == 4'h9
		| io_address[3:0] == 4'ha | io_address[3:0] == 4'hc | io_address[3:0] == 4'he)) |
		(io_address[15:4] == (gusbase[15:4] | 16'h10)
		& (io_address[3:0] == 4'h2 | io_address[3:0] == 4'h3 | io_address[3:0] == 4'h4
		| io_address[3:0] == 4'h5 | io_address[3:0] == 4'h7));
	
	wire ismixeraddr = io_address == gusbase;
	wire isdmairqaddr = io_address == (gusbase | 16'hb);
	
	reg dmairq_regsel;
	reg dmairq_enable;
	wire DREQ2 = dmairq_enable & DREQ;
	wire IRQE = dmairq_enable & IRQ;
	
	reg [3:0] irqsel;
	reg [3:0] dmasel;
	
	assign LPC_DATA = lpc_state ? lpc_out : 'hz;
	assign SERIRQ = sr_state ? sr_value : 'hz;
	
	wire goodaddr = isgf1addr | ismixeraddr | isdmairqaddr;
	
	gf1 gf1(
		.MCLK(CLK),
		.CLK(gf1_clk2),
		.IOW(IOW),
		.IOR(IOR),
		.ADDRESS(io_address[3:0]),
		.DATA_i(wr_data),
		.DATA_o(rd_data_gf1),
		.IO16(IO16),
		.CS1(isgf1addr),
		.CS2(0),
		.DRQ1(DREQ),
		.DACK1(DACK1),
		.DACK2(0),
		.IRQ1(IRQ1),
		.IRQ2(IRQ2),
		.RESET(reset),
		.DMA_TC(io_dma_tc),
		.DRAM_CAS0(CAS),
		.DRAM_RAS(RAS),
		.DRAM_ADDR(DADDR),
		.DRAM_DATA_i(DRAM_i),
		.DRAM_DATA_o(DRAM_o),
		.DRAM_WE(DRAM_WE),
		.DAC_CLK(dac_clk),
		.DAC_DATA(dac_data),
		.DAC_LR(dac_lr),
		.WAIT(gf1_wait)
		);
	
	always @(posedge CLK)
	begin
		// dram
		if (RAS & ~o_RAS)
			DADDR_l[17:9] <= DADDR;
		if (CASA & ~o_CAS)
		begin
			DADDR_l[8:0] <= DADDR;
			DWE <= DRAM_WE;
		end
		if (CASA & o_CAS)
		begin
			if (DWE)
				dram[DADDR_l] <= DRAM_o;
			else
				DRAM_i <= dram[DADDR_l];
		end
		
		o_CAS <= CASA;
		o_RAS <= RAS;
		
		// dac
		
		if (dac_clk & ~o_dac_clk)
		begin
			dac_shifter = { dac_shifter[14:0], dac_data };
			if (o_dac_lr & ~dac_lr)
				dac_left <= dac_shifter;
			else if (~o_dac_lr & dac_lr)
				dac_right <= dac_shifter;
			
			o_dac_lr <= dac_lr;
		end
		
		o_dac_clk <= dac_clk;
		
		// lpc
		if (reset)
		begin
			lpc_cnt <= 0;
			io_write <= 0;
			io_read <= 0;
			io_dmawrite <= 0;
			io_dmaread <= 0;
			lpc_state <= 0;
			lpc_out <= 0;
			dmairq_regsel <= 0;
			irqsel <= 7;
			dmasel <= 1;
			IO16 <= 0;
			IOW <= 0;
			IOR <= 0;
			io_dma_tc <= 0;
			o_DREQ <= 0;
			dma_req <= 0;
			o_IRQ <= 0;
			irq_sleep <= 0;
			irq_state <= 0;
			sr_state <= 0;
			sr_value <= 0;
			sr_clear <= 0;
			LPC_DREQ <= 1;
			dmairq_enable <= 0;
		end
		else
		begin
			if (~LPC_FRM & LPC_DATA == 4'h0)
			begin
				lpc_cnt <= 1;
			end
			else if (lpc_cnt == 1)
			begin
				io_write <= LPC_DATA == 4'h2;
				io_read <= LPC_DATA == 4'h0;
				io_dmaread <= LPC_DATA == 4'h8;
				io_dmawrite <= LPC_DATA == 4'ha;
				lpc_cnt <= 2;
				IOW <= 0;
				IOR <= 0;
			end
			else if (io_write)
			begin
				if (lpc_cnt == 6 & ~goodaddr) // ignore
					lpc_cnt <= 0;
				else if (lpc_cnt == 15)
					lpc_cnt <= 0;
				else
					lpc_cnt <= lpc_cnt + 1;
				case (lpc_cnt)
					2: io_address[15:12] <= LPC_DATA;
					3: io_address[11:8] <= LPC_DATA;
					4: io_address[7:4] <= LPC_DATA;
					5: io_address[3:0] <= LPC_DATA;
					6: wr_data[3:0] <= LPC_DATA;
					7: wr_data[7:4] <= LPC_DATA;
					// 8, 9 TAR
					9: begin lpc_state <= 1; lpc_out <= 4'h6; end
					12: begin if (isgf1addr & gf1_wait) lpc_cnt <= 12; end
					13: lpc_out <= 4'h0;
					14: lpc_out <= 4'hf;
					15: begin lpc_state <= 0; end
				endcase
				if (isgf1addr)
				begin
					case (lpc_cnt)
						9: IOW <= 1;
						14: IOW <= 0;
					endcase
				end
				if (ismixeraddr)
				begin
					if (lpc_cnt == 9)
					begin
						dmairq_regsel <= wr_data[6];
						dmairq_enable <= wr_data[3];
					end
				end
				if (isdmairqaddr)
				begin
					if (lpc_cnt == 9)
					begin
						if (dmairq_regsel)
						begin
							case (wr_data[2:0])
								3'h0: irqsel <= 4'h0;
								3'h1: irqsel <= 4'h2;
								3'h2: irqsel <= 4'h5;
								3'h3: irqsel <= 4'h3;
								3'h4: irqsel <= 4'h7;
								3'h5: irqsel <= 4'hb;
								3'h6: irqsel <= 4'hc;
								3'h7: irqsel <= 4'hf;
							endcase
						end
						else
						begin
							case (wr_data[2:0])
								3'h0: dmasel <= 4'h0;
								3'h1: dmasel <= 4'h1;
								3'h2: dmasel <= 4'h3;
								3'h3: dmasel <= 4'h5;
								3'h4: dmasel <= 4'h6;
								3'h5: dmasel <= 4'h7;
								default: dmasel <= 4'h0;
							endcase
						end
					end
				end
			end
			else if (io_read)
			begin
				if (lpc_cnt == 6 &  ~goodaddr) // ignore
					lpc_cnt <= 0;
				else if (lpc_cnt == 15)
					lpc_cnt <= 0;
				else
					lpc_cnt <= lpc_cnt + 1;
				case (lpc_cnt)
					2: io_address[15:12] <= LPC_DATA;
					3: io_address[11:8] <= LPC_DATA;
					4: io_address[7:4] <= LPC_DATA;
					5: io_address[3:0] <= LPC_DATA;
					// 6, 7 TAR
					7: begin lpc_state <= 1; lpc_out <= 4'h6; end
					10: begin if (isgf1addr & gf1_wait) lpc_cnt <= 10; end
					11: begin lpc_out <= 4'h0; end
					12: lpc_out <= rd_data[3:0];
					13: lpc_out <= rd_data[7:4];
					14: begin lpc_out <= 4'hf; end
					15: begin lpc_state <= 0; end
				endcase
				if (isgf1addr)
				begin
					case (lpc_cnt)
						6: IOR <= 1;
						11: rd_data <= rd_data_gf1;
						15: IOR <= 0;
					endcase
				end
			end
			else if (io_dmaread)
			begin
				if (lpc_cnt == 4 & (io_dma_chan != dmasel | io_dma_size[1])) // ignore
					lpc_cnt <= 0;
				else if (lpc_cnt == 15)
					lpc_cnt <= 0;
				else if (lpc_cnt == 5 & ~io_dma_size[0])
					lpc_cnt <= 8;
				else
					lpc_cnt <= lpc_cnt + 1;
				case (lpc_cnt)
					2: begin io_dma_chan <= LPC_DATA[2:0]; io_dma_tc <= LPC_DATA[3]; end
					3: io_dma_size <= LPC_DATA[1:0];
					4: wr_data[3:0] <= LPC_DATA;
					5: wr_data[7:4] <= LPC_DATA;
					6: wr_data[11:8] <= LPC_DATA;
					7: wr_data[15:12] <= LPC_DATA;
					// 8, 9 TAR
					9: begin lpc_state <= 1; lpc_out <= 4'h6; end
					12: begin if ((io_dma_chan == dmasel) & gf1_wait) lpc_cnt <= 12; end
					13: begin lpc_out <= 4'h0; o_DREQ <= 1'h0; io_dma_tc <= 0; end
					14: lpc_out <= 4'hf;
					15: begin lpc_state <= 0; end
				endcase
				if (io_dma_chan == dmasel)
				begin
					case (lpc_cnt)
						8: begin IOW <= 1; IO16 <= io_dma_size[0]; DACK1 <= 1; end
						13: begin IOW <= 0; IO16 <= 0; DACK1 <= 0; end
					endcase
				end
			end
			else if (io_dmawrite)
			begin
				if (lpc_cnt == 4 & (io_dma_chan != dmasel | io_dma_size[1])) // ignore
					lpc_cnt <= 0;
				else if (lpc_cnt == 15)
					lpc_cnt <= 0;
				else if (lpc_cnt == 5 & ~io_dma_size[0])
					lpc_cnt <= 8;
				else
					lpc_cnt <= lpc_cnt + 1;
				case (lpc_cnt)
					2: begin io_dma_chan <= LPC_DATA[2:0]; io_dma_tc <= LPC_DATA[3]; end
					3: io_dma_size <= LPC_DATA[1:0];
					// 4, 5 TAR
					5: begin lpc_state <= 1; lpc_out <= 4'h6; end
					8: begin if ((io_dma_chan == dmasel) & gf1_wait) lpc_cnt <= 8; end
					9: begin lpc_out <= 4'h0; o_DREQ <= 1'h0; io_dma_tc <= 0; end
					10: lpc_out <= rd_data[3:0];
					11: lpc_out <= rd_data[7:4];
					10: lpc_out <= rd_data[11:8];
					11: lpc_out <= rd_data[15:12];
					14: begin lpc_out <= 4'hf; end
					15: begin lpc_state <= 0; end
				endcase
				if (io_dma_chan == dmasel)
				begin
					case (lpc_cnt)
						4: begin IOR <= 1; IO16 <= io_dma_size[0]; DACK1 <= 1; end
						9: begin rd_data <= rd_data_gf1; IOR <= 0; IO16 <= 0; DACK1 <= 0; end
					endcase
				end
			end
			else
				lpc_cnt <= 0;
			
			if (DREQ2 != o_DREQ & ~io_dmaread & ~io_dmawrite)
			begin
				if (dma_req == 0)
				begin
					dma_req <= 1;
					o_DREQ <= DREQ2;
				end
			end
			
			if (dma_req == 1)
			begin
				LPC_DREQ <= 0;
				dma_req <= 2;
			end
			else if (dma_req == 2)
			begin
				LPC_DREQ <= dmasel[2];
				dma_req <= 3;
			end
			else if (dma_req == 3)
			begin
				LPC_DREQ <= dmasel[1];
				dma_req <= 4;
			end
			else if (dma_req == 4)
			begin
				LPC_DREQ <= dmasel[0];
				dma_req <= 5;
			end
			else if (dma_req == 5)
			begin
				LPC_DREQ <= o_DREQ;
				dma_req <= 6;
			end
			else if (dma_req == 6)
			begin
				LPC_DREQ <= 1;
				dma_req <= 0;
			end
			
			if (sr_clear)
			begin
				sr_state <= 0;
				sr_clear <= 0;
			end
			
			if (IRQE != o_IRQ)
			begin
				if (irq_sleep & irq_state == 0)
				begin
					sr_state <= 1;
					sr_value <= 0;
					sr_clear <= 1;
					irq_sleep <= 0;
					o_IRQ <= IRQE;
				end
			end
			
			if (irq_state == 0)
			begin
				if (~SERIRQ)
					irq_state <= 1;
			end
			else if (irq_state == 1)
			begin
				if (SERIRQ)
					irq_state <= 255;
			end
			else if (irq_state == 100)
			begin
				irq_state <= 101;
			end
			else if (irq_state == 101)
			begin
				irq_sleep <= SERIRQ;
				irq_state <= 102;
			end
			else if (irq_state == 102)
			begin
				irq_state <= 103;
			end
			else if (irq_state == 103)
			begin
				irq_state <= 0;
			end
			else if (irq_state == 255 - irqsel * 3)
			begin
				sr_state <= 1;
				sr_value <= o_IRQ;
				irq_state <= irq_state - 1;
			end
			else if (irq_state == 254 - irqsel * 3)
			begin
				sr_value <= 1;
				irq_state <= irq_state - 1;
			end
			else if (irq_state == 253 - irqsel * 3)
			begin
				sr_state <= 0;
				irq_state <= irq_state - 1;
			end
			else if (irq_state == 255 - 48)
			begin
				if (~SERIRQ)
					irq_state <= 100;
			end
			else
			begin
				irq_state <= irq_state - 1;
			end
		end
		
		
	end

endmodule
