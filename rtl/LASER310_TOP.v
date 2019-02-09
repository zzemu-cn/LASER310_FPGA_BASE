`timescale 1 ns / 1 ns

// 选择开发板
//`define	DE0
//`define	DE1
//`define	DE2
//`define	DE2_70
`define	DE2_115

// 选择 Z80 软核
`define	TV80
//`define	NEXTZ80

`ifdef DE0

`define FPGA_ALTERA
`define FPGA_ALTERA_C3
`define CLOCK_50MHZ
`define VGA_RESISTOR
`define VGA_BIT12
`define AUDIO_GPIO
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_16K
`define VRAM_2K

`endif


`ifdef DE1

`define FPGA_ALTERA
`define FPGA_ALTERA_C2
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define VGA_RESISTOR
`define VGA_BIT12
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_4K
`define VRAM_2K

`endif


`ifdef DE2

`define FPGA_ALTERA
`define FPGA_ALTERA_C2
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define VGA_ADV7123
`define VGA_BIT30
`define UART_CHIP
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_16K
`define VRAM_2K

`endif


`ifdef DE2_70

`define FPGA_ALTERA
`define FPGA_ALTERA_C2
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define VGA_ADV7123
`define VGA_BIT30
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_16K
`define VRAM_2K

`endif


`ifdef DE2_115

`define FPGA_ALTERA
`define FPGA_ALTERA_C4
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define VGA_ADV7123
`define VGA_BIT24
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_16K
`define VRAM_2K

`endif




module LASER310_TOP(
	CLK50MHZ,

`ifdef CLOCK_27MHZ
	CLK27MHZ,
`endif

	// VGA
`ifdef VGA_RESISTOR
	VGA_RED,
	VGA_GREEN,
	VGA_BLUE,
	VGA_HS,
	VGA_VS,
`endif

`ifdef VGA_ADV7123
	VGA_DAC_RED,
	VGA_DAC_GREEN,
	VGA_DAC_BLUE,
	VGA_HS,
	VGA_VS,
	VGA_DAC_BLANK_N,
	VGA_DAC_SYNC_N,
	VGA_DAC_CLOCK,
`endif

	// PS/2
	PS2_KBCLK,
	PS2_KBDAT,

	LED,

`ifdef AUDIO_WM8731
	////////////////	Audio CODEC		////////////////////////
	AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
	AUD_ADCDAT,						//	Audio CODEC ADC Data
	AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
	AUD_DACDAT,						//	Audio CODEC DAC Data
	AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
	AUD_XCK,						//	Audio CODEC Chip Clock

	////////////////////	I2C		////////////////////////////
	I2C_SDAT,						//	I2C Data
	I2C_SCLK,						//	I2C Clock

`endif

`ifdef GPIO_PIN
	////////////////////	GPIO	////////////////////////////
	//GPIO_0,							//	GPIO Connection 0
	//GPIO_1,							//	GPIO Connection 1
`endif

	BUTTON_N
);


input				CLK50MHZ;

`ifdef CLOCK_27MHZ
input				CLK27MHZ;
`endif


`ifdef VGA_RESISTOR
// de0 de1
`ifdef VGA_BIT12
output	wire	[3:0]	VGA_RED;
output	wire	[3:0]	VGA_GREEN;
output	wire	[3:0]	VGA_BLUE;
`endif

output	wire			VGA_HS;
output	wire			VGA_VS;
`endif


`ifdef VGA_ADV7123
// de2
`ifdef VGA_BIT30
output	wire	[9:0]	VGA_DAC_RED;
output	wire	[9:0]	VGA_DAC_GREEN;
output	wire	[9:0]	VGA_DAC_BLUE;
`endif

`ifdef VGA_BIT24
output	wire	[7:0]	VGA_DAC_RED;
output	wire	[7:0]	VGA_DAC_GREEN;
output	wire	[7:0]	VGA_DAC_BLUE;
`endif

output	reg				VGA_HS;
output	reg				VGA_VS;
output	wire			VGA_DAC_BLANK_N;
output	wire			VGA_DAC_SYNC_N;
output	wire			VGA_DAC_CLOCK;
`endif


// PS/2
input 			PS2_KBCLK;
input			PS2_KBDAT;

// LEDs
output	wire	[9:0]	LED;


`ifdef AUDIO_WM8731

////////////////////	Audio CODEC		////////////////////////////
// ADC
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data

// DAC
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output	wire	AUD_DACDAT;				//	Audio CODEC DAC Data

inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output	wire	AUD_XCK;				//	Audio CODEC Chip Clock

////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock

`endif


input	[1:0]		BUTTON_N;
wire	[1:0]		BUTTON;
assign	BUTTON	=	~BUTTON_N;


`ifdef GPIO_PIN
////////////////////////	GPIO	////////////////////////////////
//inout	[35:0]	GPIO_0;		//	GPIO Connection 0
//inout	[35:0]	GPIO_1;		//	GPIO Connection 1
//output	[35:0]	GPIO_0;		//	GPIO Connection 0
//output	[35:0]	GPIO_1;		//	GPIO Connection 1
`endif


// 10MHz 的频率用于模块计数， 包括产生 50HZ 的中断信号的时钟，uart 模块的时钟，模拟磁带模块的时钟
// 选择 10MHz 是因为 Cyclone II 的DLL 分频最多能到 5。最初打算用 1MHz。
wire				CLK10MHZ;
CLK10MHZ_PLL CLK10MHZ_PLL_INST(
	CLK50MHZ,
	CLK10MHZ);

// CLOCK & BUS
wire				BASE_CLK = CLK50MHZ;

reg		[3:0]		CLK;

reg					MEM_OP_WR;
//reg					MEM_RD;

// 50% 方波信号, 引出到 GPIO 端口
reg					GPIO_CPU_CLK;

// Processor
reg					CPU_CLK;
wire	[15:0]		CPU_A;
wire	[7:0]		CPU_DI;
wire	[7:0]		CPU_DO;

wire				CPU_RESET;
wire				CPU_HALT;
wire				CPU_WAIT;

wire				CPU_MREQ;
wire				CPU_RD;
wire				CPU_WR;
wire				CPU_IORQ;

reg					CPU_INT;
wire				CPU_NMI;
wire				CPU_M1;


wire				CPU_BUSRQ;
wire				CPU_BUSAK;

wire				CPU_RFSH;

`ifdef TV80

wire				CPU_RESET_N;
wire				CPU_HALT_N;
wire				CPU_WAIT_N;

wire				CPU_MREQ_N;
wire				CPU_RD_N;
wire				CPU_WR_N;
wire				CPU_IORQ_N;

wire				CPU_INT_N;
wire				CPU_NMI_N;
wire				CPU_M1_N;

wire				CPU_BUSRQ_N;
wire				CPU_BUSAK_N;

wire				CPU_RFSH_N;

`endif


// VRAM
wire	[12:0]		VRAM_ADDRESS;
wire				VRAM_WR;
wire	[7:0]		VRAM_DATA_OUT;

wire				VDG_RD;
wire	[12:0]		VDG_ADDRESS;
wire	[7:0]		VDG_DATA;

// ROM IO RAM
wire	[7:0]		SYS_ROM_DATA;

wire				RAM_78_WR;
wire	[7:0]		RAM_78_DATA_OUT;

wire				RAM_80_WR;
wire	[7:0]		RAM_80_DATA_OUT;


wire				RAM_16K_WR;
wire	[7:0]		RAM_16K_DATA_OUT;

wire				ADDRESS_ROM;
wire				ADDRESS_IO;
wire				ADDRESS_VRAM;

wire				ADDRESS_RAM_78;
wire				ADDRESS_RAM_80;

wire				ADDRESS_RAM_16K;


/*
74LS174输出的各个控制信号是：
Q5 蜂鸣器B端电平
Q4 IC15（6847）第39脚的CSS信号（控制显示基色）
Q3 IC15（6847）第35脚的~A/G信号（控制显示模式）
Q2 磁带记录信号电平
Q1 未用
Q0 蜂鸣器A端电平
*/

reg		[7:0]		LATCHED_IO_DATA_WR;

// VGA
wire	[7:0]		VGA_OUT_RED;
wire	[7:0]		VGA_OUT_GREEN;
wire	[7:0]		VGA_OUT_BLUE;

wire				VGA_OUT_HS;
wire				VGA_OUT_VS;

wire				VGA_OUT_BLANK;

`ifdef CLOCK_50MHZ
reg					VGA_CLK;
`else
// 通过 PLL 生成
wire				VGA_CLK;
`endif

// keyboard
reg		[4:0]		KB_CLK;

wire	[7:0]		SCAN;
wire				PRESS;
wire				PRESS_N;
wire				EXTENDED;

reg		[63:0]		KEY;
reg		[9:0]		KEY_EX;
reg		[11:0]		KEY_Fxx;
wire	[7:0]		KEY_DATA;
//reg	[63:0]		LAST_KEY;
//reg				CAPS_CLK;
//reg				CAPS;
wire				A_KEY_PRESSED;

reg		[7:0]		LATCHED_KEY_DATA;

// speaker

wire	SPEAKER_A = LATCHED_IO_DATA_WR[0];
wire	SPEAKER_B = LATCHED_IO_DATA_WR[5];

// cassette

wire	[1:0]		CASS_OUT;
wire				CASS_IN;
wire				CASS_IN_L;
wire				CASS_IN_R;



// other
wire				SYS_RESET_N;
wire				RESET_N;
wire				RESET_AHEAD_N;

reg		[16:0]		RESET_KEY_COUNT;
wire				RESET_KEY_N;


//	All inout port turn to tri-state
//assign	DRAM_DQ		=	16'hzzzz;
//assign	FL_DQ		=	8'hzz;
//assign	SRAM_DQ		=	16'hzzzz;
//assign	SD_DAT		=	1'bz;
assign	I2C_SDAT	=	1'bz;
assign	AUD_ADCLRCK	=	1'bz;
assign	AUD_DACLRCK	=	1'bz;
assign	AUD_BCLK	=	1'bz;


// reset

assign SYS_RESET_N = !BUTTON[0];

RESET_DE RESET_DE(
	.CLK(CLK50MHZ),			// 50MHz
	.SYS_RESET_N(SYS_RESET_N && RESET_KEY_N),
	.RESET_N(RESET_N),		// 50MHz/32/65536
	.RESET_AHEAD_N(RESET_AHEAD_N)	// 提前恢复，可以接 FL_RESET_N
);


// 键盘 ctrl + f12 系统复位
assign RESET_KEY_N = RESET_KEY_COUNT[16];


`ifdef SIMULATE
initial
	begin
		VGA_CLK = 1'b0;
		CLK = 4'b0;
	end
`endif

`ifdef CLOCK_50MHZ

always @(negedge CLK50MHZ)
	VGA_CLK <= !VGA_CLK;

`endif


`ifdef CLOCK_27MHZ

VGA_Audio_PLL  VGA_AUDIO_PLL(.inclk0(CLK27MHZ),.c0(VGA_CLK),.c1(AUD_CTRL_CLK));

`endif


// 频率 50HZ
// 回扫周期暂定为：2线 x 800点 x 10MHZ / 25MHZ

// ~FS 垂直同步信号，送往IC1、IC2称IC4。6847对CPU的唯一直接影响，便是它的~FS输出被作为Z80A的~INT控制信号；
// 每一场扫描结束，6847的~FS信号变低，便向Z80A发出中断请求。在PAL制中，场频为50Hz，每秒就有50次中断请求，以便系统程序利用场消隐期运行监控程序，访问显示RAM。

// 在加速模式中，要考虑对该计数器的影响

// 系统中断：简化处理是直接接到 VGA 的垂直回扫信号，频率60HZ。带来的问题是软件计时器会产生偏差。

reg 		[17:0]	INT_CNT;

always @ (negedge CLK10MHZ)
	case(INT_CNT[17:0])
		18'd0:
		begin
			CPU_INT <= 1'b1;
			INT_CNT <= 18'd1;
		end
		18'd640:
		begin
			CPU_INT <= 1'b0;
			INT_CNT <= 18'd641;
		end
		18'd199999:
		begin
			INT_CNT <= 18'd0;
		end
		default:
		begin
			INT_CNT <= INT_CNT + 1;
		end
	endcase

// CPU clock

// 17.7MHz/5 = 3.54MHz
// LASER310 CPU：Z-80A/3.54MHz
// VZ300 CPU：Z-80A/3.54MHz

// 正常速度 50MHZ / 14 = 3.57MHz

// 同步内存操作
// 写 0 CPU 写信号和地址 1 锁存写和地址 2 完成写操作
// 读 0 CPU 读信号和地址 1 锁存读和地址 2 完读写操作，开始输出数据

// 读取需要中间间隔一个时钟


`ifdef SIMULATE
initial
	begin
		CLK = 4'b0;
	end
`endif

always @(posedge BASE_CLK or negedge RESET_N)
	if(~RESET_N)
	begin
		CPU_CLK					<=	1'b0;
		GPIO_CPU_CLK			<=	1'b0;


		MEM_OP_WR				<=	1'b0;

		LATCHED_KEY_DATA		<=	8'b0;
		LATCHED_IO_DATA_WR		<=	8'b0;

		CLK						<=	4'd0;
	end
	else
	begin
		case (CLK[3:0])
		4'd0:
			begin
				// 同步内存，等待读写信号建立
				CPU_CLK				<=	1'b1;
				GPIO_CPU_CLK		<=	1'b1;

				MEM_OP_WR			<=	1'b1;

				CLK					<=	4'd1;
			end

		4'd1:
			begin
				// 同步内存，锁存读写信号和地址
				CPU_CLK				<=	1'b0;
				MEM_OP_WR			<=	1'b0;

				LATCHED_KEY_DATA	<=	KEY_DATA;

				if({CPU_MREQ,CPU_RD,CPU_WR,ADDRESS_IO}==4'b1011)
					LATCHED_IO_DATA_WR	<=	CPU_DO;

				CLK					<=	4'd2;
			end

		4'd2:
			begin
				// 完成读写操作，开始输出
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	4'd3;
			end


		4'd7:
			begin
				CPU_CLK				<=	1'b0;
				GPIO_CPU_CLK		<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	4'd8;
			end

		// 正常速度
		4'd13:
			begin
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	4'd0;
			end
		default:
			begin
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	CLK + 1'b1;
			end
		endcase
	end

	//vga_pll vgapll(CLK50MHZ, VGA_CLOCK);
	/* This module generates a clock with half the frequency of the input clock.
	 * For the VGA adapter to operate correctly the clock signal 'clock' must be
	 * a 50MHz clock. The derived clock, which will then operate at 25MHz, is
	 * required to set the monitor into the 640x480@60Hz display mode (also known as
	 * the VGA mode).
	 */


wire [7:0] InPort = 8'b0;

// CPU

`ifdef NEXTZ80

// 输入控制信号 RESET_N INT_N NMI_N WAIT_N BUSRQ_N DI

NextZ80 Z80CPU (
	.DI(CPU_IORQ ? (CPU_M1 ? 8'b00000000 : InPort) : CPU_DI),
	.DO(CPU_DO),
	.ADDR(CPU_A),
	.WR(CPU_WR),
	.MREQ(CPU_MREQ),
	.IORQ(CPU_IORQ),
	.HALT(CPU_HALT),
	.CLK(CPU_CLK),
	.RESET(CPU_RESET),
	.INT(CPU_INT),
	.NMI(CPU_NMI),
	.WAIT(CPU_WAIT),
	.M1(CPU_M1)
);

`endif


`ifdef TV80

assign CPU_M1 = ~CPU_M1_N;
assign CPU_MREQ = ~CPU_MREQ_N;
assign CPU_IORQ = ~CPU_IORQ_N;
assign CPU_RD = ~CPU_RD_N;
assign CPU_WR = ~CPU_WR_N;
assign CPU_RFSH = ~CPU_RFSH_N;
assign CPU_HALT= ~CPU_HALT_N;
assign CPU_BUSAK = ~CPU_BUSAK_N;

assign CPU_RESET_N = ~CPU_RESET;
assign CPU_WAIT_N = ~CPU_WAIT;
assign CPU_INT_N = ~CPU_INT;	// 50HZ
//assign CPU_INT_N = ~VGA_VS;	// 接 VGA 垂直回扫信号 60HZ
assign CPU_NMI_N = ~CPU_NMI;
assign CPU_BUSRQ_N = ~CPU_BUSRQ;

/*
  // Outputs
  m1_n, mreq_n, iorq_n, rd_n, wr_n, rfsh_n, halt_n, busak_n, A, dout,
  // Inputs
  reset_n, clk, wait_n, int_n, nmi_n, busrq_n, di
*/

tv80s Z80CPU (
	.m1_n(CPU_M1_N),
	.mreq_n(CPU_MREQ_N),
	.iorq_n(CPU_IORQ_N),
	.rd_n(CPU_RD_N),
	.wr_n(CPU_WR_N),
	.rfsh_n(CPU_RFSH_N),
	.halt_n(CPU_HALT_N),
	.busak_n(CPU_BUSAK_N),
	.A(CPU_A),
	.dout(CPU_DO),
	.reset_n(CPU_RESET_N),
	.clk(CPU_CLK),
	.wait_n(CPU_WAIT_N),
	.int_n(CPU_INT_N),
	.nmi_n(CPU_NMI_N),
	.busrq_n(CPU_BUSRQ_N),
	.di(CPU_IORQ_N ? CPU_DI : (CPU_M1_N ? InPort: 8'b00000000))
);

`endif

assign CPU_RESET = ~RESET_N;

assign CPU_NMI = 1'b0;

// LASER310 的 WAIT_N 始终是高电平。
assign CPU_WAIT = 1'b0;

//assign CPU_WAIT = CPU_MREQ && (~CLKStage[2]);


// 0000 -- 3FFF ROM 16KB
// 4000 -- 5FFF DOS
// 6000 -- 67FF BOOT ROM
// 6800 -- 6FFF I/O
// 7000 -- 77FF VRAM 2KB (SRAM 6116)
// 7800 -- 7FFF RAM 2KB
// 8000 -- B7FF RAM 14KB
// B800 -- BFFF RAM ext 2KB
// C000 -- F7FF RAM ext 14KB

assign ADDRESS_ROM			=	(CPU_A[15:14] == 2'b00)?1'b1:1'b0;
assign ADDRESS_IO			=	(CPU_A[15:11] == 5'b01101)?1'b1:1'b0;
assign ADDRESS_VRAM			=	(CPU_A[15:11] == 5'b01110)?1'b1:1'b0;

// 7800 -- 7FFF RAM 2KB
assign ADDRESS_RAM_78		=	(CPU_A[15:11] == 5'b01111)?1'b1:1'b0;

// 8000 -- 87FF RAM 2KB
assign ADDRESS_RAM_80		=	(CPU_A[15:11] == 5'b10000)?1'b1:1'b0;


// 7800 -- 7FFF RAM 2KB
// 8000 -- B7FF RAM 14KB

assign ADDRESS_RAM_16K		=	(CPU_A[15:12] == 4'h8)?1'b1:
								(CPU_A[15:12] == 4'h9)?1'b1:
								(CPU_A[15:12] == 4'hA)?1'b1:
								(CPU_A[15:11] == 5'b01111)?1'b1:
								(CPU_A[15:11] == 5'b10110)?1'b1:
								1'b0;


assign VRAM_WR			= ({ADDRESS_VRAM,MEM_OP_WR,CPU_WR,CPU_IORQ} == 4'b1110)?1'b1:1'b0;

assign RAM_78_WR		= ({ADDRESS_RAM_78,MEM_OP_WR,CPU_WR,CPU_IORQ} == 4'b1110)?1'b1:1'b0;
assign RAM_80_WR		= ({ADDRESS_RAM_80,MEM_OP_WR,CPU_WR,CPU_IORQ} == 4'b1110)?1'b1:1'b0;

assign RAM_16K_WR		= ({ADDRESS_RAM_16K,MEM_OP_WR,CPU_WR,CPU_IORQ} == 4'b1110)?1'b1:1'b0;


`ifdef	RAM_ON_FPGA

assign CPU_DI = 	ADDRESS_ROM			? SYS_ROM_DATA		:
					ADDRESS_IO			? LATCHED_KEY_DATA	:
					ADDRESS_VRAM		? VRAM_DATA_OUT		:
`ifdef BASE_RAM_2K
					ADDRESS_RAM_78		? RAM_78_DATA_OUT	:
`endif
`ifdef BASE_RAM_4K
					ADDRESS_RAM_78		? RAM_78_DATA_OUT	:
					ADDRESS_RAM_80		? RAM_80_DATA_OUT	:
`endif
`ifdef BASE_RAM_16K
					ADDRESS_RAM_16K		? RAM_16K_DATA_OUT	:
`endif
					8'hzz;

`endif


`ifdef BASE_SYS_ROM

`ifdef FPGA_ALTERA

sys_rom_altera sys_rom(
	.address(CPU_A[13:0]),
	.clock(BASE_CLK),
	.q(SYS_ROM_DATA)
);

`endif

`endif


`ifdef	RAM_ON_FPGA


`ifdef BASE_RAM_2K

`ifdef FPGA_ALTERA

ram_2k_altera sys_ram_78(
	.address(CPU_A[10:0]),
	.clock(BASE_CLK),
	.data(CPU_DO),
	.wren(CPU_MREQ & RAM_78_WR),
	.q(RAM_78_DATA_OUT)
);

`endif

`endif


`ifdef BASE_RAM_4K

`ifdef FPGA_ALTERA

ram_2k_altera sys_ram_78(
	.address(CPU_A[10:0]),
	.clock(BASE_CLK),
	.data(CPU_DO),
	.wren(CPU_MREQ & RAM_78_WR),
	.q(RAM_78_DATA_OUT)
);

ram_2k_altera sys_ram_80(
	.address(CPU_A[10:0]),
	.clock(BASE_CLK),
	.data(CPU_DO),
	.wren(CPU_MREQ & RAM_80_WR),
	.q(RAM_80_DATA_OUT)
);

`endif

`endif


`ifdef BASE_RAM_16K

`ifdef FPGA_ALTERA

ram_altera sys_ram_16k(
	.address(CPU_A[13:0]),
	.clock(BASE_CLK),
	.data(CPU_DO),
	.wren(CPU_MREQ & RAM_16K_WR),
	.q(RAM_16K_DATA_OUT)
);

`endif

`endif


`endif


/*****************************************************************************
* Video
******************************************************************************/
// Request for every other line to be black
// Looks more like the original video


`ifdef VRAM_2K

`ifdef FPGA_ALTERA

vram_altera vram_2k(
	.address_a(CPU_A[10:0]),
	.address_b(VDG_ADDRESS[10:0]),
	.clock_a(BASE_CLK),
	.clock_b(VDG_RD),
	.data_a(CPU_DO),
	.data_b(),
	.wren_a(CPU_MREQ & VRAM_WR),
	.wren_b(1'b0),
	.q_a(VRAM_DATA_OUT),
	.q_b(VDG_DATA)
);

`endif

`endif


// Video timing and modes
MC6847_VGA MC6847_VGA(
	.PIX_CLK(VGA_CLK),		//25 MHz = 40 nS
	.RESET_N(RESET_N),

	.RD(VDG_RD),
	.DD(VDG_DATA),
	.DA(VDG_ADDRESS),

	.AG(LATCHED_IO_DATA_WR[3]),
	.AS(1'b0),
	.EXT(1'b0),
	.INV(1'b0),
	.GM(3'b010),
	.CSS(LATCHED_IO_DATA_WR[4]),

	// vga
	.VGA_OUT_HSYNC(VGA_OUT_HS),
	.VGA_OUT_VSYNC(VGA_OUT_VS),
	.VGA_OUT_RED(VGA_OUT_RED),
	.VGA_OUT_GREEN(VGA_OUT_GREEN),
	.VGA_OUT_BLUE(VGA_OUT_BLUE)
);


`ifdef VGA_RESISTOR

`ifdef VGA_BIT12
assign VGA_RED = VGA_OUT_RED[7:4];
assign VGA_GREEN = VGA_OUT_GREEN[7:4];
assign VGA_BLUE = VGA_OUT_BLUE[7:4];
`endif

assign VGA_HS = VGA_OUT_HS;
assign VGA_VS = VGA_OUT_VS;

`endif

`ifdef VGA_ADV7123

`ifdef VGA_BIT24
assign VGA_DAC_RED = VGA_OUT_RED;
assign VGA_DAC_GREEN = VGA_OUT_GREEN;
assign VGA_DAC_BLUE = VGA_OUT_BLUE;
`endif

`ifdef VGA_BIT30
assign VGA_DAC_RED = {VGA_OUT_RED,2'b0};
assign VGA_DAC_GREEN = {VGA_OUT_GREEN,2'b0};
assign VGA_DAC_BLUE = {VGA_OUT_BLUE,2'b0};
`endif

assign VGA_DAC_BLANK_N = 1'b1;
assign VGA_DAC_SYNC_N = ~(VGA_OUT_HS | VGA_OUT_VS);
assign VGA_DAC_CLOCK = VGA_CLK;

always @(posedge VGA_CLK)
	begin
		VGA_HS <= VGA_OUT_HS;
		VGA_VS <= VGA_OUT_VS;
	end

`endif


// keyboard

/*****************************************************************************
* Convert PS/2 keyboard to ASCII keyboard
******************************************************************************/

/*
   KD5 KD4 KD3 KD2 KD1 KD0 扫描用地址
A0  R   Q   E       W   T  68FEH       0
A1  F   A   D  CTRL S   G  68FDH       8
A2  V   Z   C  SHFT X   B  68FBH      16
A3  4   1   3       2   5  68F7H      24
A4  M  空格 ，      .   N  68EFH      32
A5  7   0   8   -   9   6  68DFH      40
A6  U   P   I  RETN O   Y  68BFH      48
A7  J   ；  K   :   L   H  687FH      56
*/

//  7: 0
// 15: 8
// 23:16
// 31:24
// 39:32
// 47:40
// 55:48
// 63:56

/*
`ifdef SIMULATE
initial
	begin
		LAST_KEY = 72'b0;
	end
`endif
*/

// 键盘检测的方法，就是循环地问每一行线发送低电平信号，也就是用该地址线为“0”的地址去读取数据。
// 例如，检测第一行时，使A0为0，其余为1；加上选通IC4的高五位地址01101，成为01101***11111110B（A8~A10不起作用，
// 可为任意值，故68FEH，69FEH，6AFEH，6BFEH，6CFEH，6DFEH，6EFEH，6FFEH均可）。
// 读 6800H 判断是否有按键按下。

// 键盘选通，整个竖列有一个选通的位置被按下，对应值为0。

// 键盘扩展
// 加入方向键盘
// left:  ctrl M      37 KEY_EX[5]
// right: ctrl ,      35 KEY_EX[6]
// up:    ctrl .      33 KEY_EX[4]
// down:  ctrl space  36 KEY_EX[7]
// esc:   ctrl -      42 KEY_EX[3]
// backspace:  ctrl M      37 KEY_EX[8]

// R-Shift


//wire KEY_CTRL_ULRD = (KEY_EX[7:4]==4'b1111);
wire KEY_CTRL_ULRD_BRK = (KEY_EX[8:3]==6'b111111);

wire KEY_DATA_BIT5 = (CPU_A[7:0]|{KEY[61], KEY[53], KEY[45],           KEY[37]&KEY_EX[5]&KEY_EX[8], KEY[29], KEY[21],           KEY[13],                   KEY[ 5]})==8'hff;
wire KEY_DATA_BIT4 = (CPU_A[7:0]|{KEY[60], KEY[52], KEY[44],           KEY[36]&KEY_EX[7],           KEY[28], KEY[20],           KEY[12],                   KEY[ 4]})==8'hff;
wire KEY_DATA_BIT3 = (CPU_A[7:0]|{KEY[59], KEY[51], KEY[43],           KEY[35]&KEY_EX[6],           KEY[27], KEY[19],           KEY[11],                   KEY[ 3]})==8'hff;
wire KEY_DATA_BIT2 = (CPU_A[7:0]|{KEY[58], KEY[50], KEY[42]&KEY_EX[3], KEY[34],                     KEY[26], KEY[18]&KEY_EX[0], KEY[10]&KEY_CTRL_ULRD_BRK, KEY[ 2]})==8'hff;
wire KEY_DATA_BIT1 = (CPU_A[7:0]|{KEY[57], KEY[49], KEY[41],           KEY[33]&KEY_EX[4],           KEY[25], KEY[17],           KEY[ 9],                   KEY[ 1]})==8'hff;
wire KEY_DATA_BIT0 = (CPU_A[7:0]|{KEY[56], KEY[48], KEY[40],           KEY[32],                     KEY[24], KEY[16],           KEY[ 8],                   KEY[ 0]})==8'hff;

/*
wire KEY_DATA_BIT5 = (CPU_A[7:0]|{KEY[61], KEY[53], KEY[45], KEY[37], KEY[29], KEY[21], KEY[13], KEY[ 5]})==8'hff;
wire KEY_DATA_BIT4 = (CPU_A[7:0]|{KEY[60], KEY[52], KEY[44], KEY[36], KEY[28], KEY[20], KEY[12], KEY[ 4]})==8'hff;
wire KEY_DATA_BIT3 = (CPU_A[7:0]|{KEY[59], KEY[51], KEY[43], KEY[35], KEY[27], KEY[19], KEY[11], KEY[ 3]})==8'hff;
wire KEY_DATA_BIT2 = (CPU_A[7:0]|{KEY[58], KEY[50], KEY[42], KEY[34], KEY[26], KEY[18], KEY[10], KEY[ 2]})==8'hff;
wire KEY_DATA_BIT1 = (CPU_A[7:0]|{KEY[57], KEY[49], KEY[41], KEY[33], KEY[25], KEY[17], KEY[ 9], KEY[ 1]})==8'hff;
wire KEY_DATA_BIT0 = (CPU_A[7:0]|{KEY[56], KEY[48], KEY[40], KEY[32], KEY[24], KEY[16], KEY[ 8], KEY[ 0]})==8'hff;
*/

wire KEY_DATA_BIT7 = 1'b1;	// 没有空置，具体用途没有理解
//wire KEY_DATA_BIT6 = CASS_IN;
wire KEY_DATA_BIT6 = ~CASS_IN;

assign KEY_DATA = { KEY_DATA_BIT7, KEY_DATA_BIT6, KEY_DATA_BIT5, KEY_DATA_BIT4, KEY_DATA_BIT3, KEY_DATA_BIT2, KEY_DATA_BIT1, KEY_DATA_BIT0 };

/*
assign KEY_DATA = 	(CPU_A[0]==1'b0) ? KEY[ 7: 0] :
					(CPU_A[1]==1'b0) ? KEY[15: 8] :
					(CPU_A[2]==1'b0) ? KEY[23:16] :
					(CPU_A[3]==1'b0) ? KEY[31:24] :
					(CPU_A[4]==1'b0) ? KEY[39:32] :
					(CPU_A[5]==1'b0) ? KEY[47:40] :
					(CPU_A[6]==1'b0) ? KEY[55:48] :
					(CPU_A[7]==1'b0) ? KEY[63:56] :
					8'hff;

assign KEY_DATA =
					(CPU_A[7]==1'b0) ? KEY[63:56] :
					(CPU_A[6]==1'b0) ? KEY[55:48] :
					(CPU_A[5]==1'b0) ? KEY[47:40] :
					(CPU_A[4]==1'b0) ? KEY[39:32] :
					(CPU_A[3]==1'b0) ? KEY[31:24] :
					(CPU_A[2]==1'b0) ? KEY[23:16] :
					(CPU_A[1]==1'b0) ? KEY[15: 8] :
					(CPU_A[0]==1'b0) ? KEY[ 7: 0] :
					8'hff;
*/


assign A_KEY_PRESSED = (KEY[63:0] == 64'hFFFFFFFFFFFFFFFF) ? 1'b0:1'b1;

always @(posedge KB_CLK[3] or negedge SYS_RESET_N)
begin
	if(~SYS_RESET_N)
	begin
		KEY					<=	64'hFFFFFFFFFFFFFFFF;
		KEY_EX				<=	10'h3FF;
		KEY_Fxx				<=	12'h000;
//		CAPS_CLK			<=	1'b0;
		RESET_KEY_COUNT		<=	17'h1FFFF;
	end
	else
	begin
		//KEY[?] <= CAPS;
		if(RESET_KEY_COUNT[16]==1'b0)
			RESET_KEY_COUNT <= RESET_KEY_COUNT+1;

		case(SCAN)
		8'h07:
		begin
				KEY_Fxx[11]	<= PRESS;	// F12 RESET
				if(PRESS && (KEY[10]==PRESS_N))
				begin
					RESET_KEY_COUNT		<=	17'h0;
				end
		end
		8'h78:	KEY_Fxx[10] <= PRESS;	// F11
		8'h09:	KEY_Fxx[ 9] <= PRESS;	// F10
		8'h01:	KEY_Fxx[ 8] <= PRESS;	// F9
		8'h0A:	KEY_Fxx[ 7] <= PRESS;	// F8
		8'h83:	KEY_Fxx[ 6] <= PRESS;	// F7
		8'h0B:	KEY_Fxx[ 5] <= PRESS;	// F6
		8'h03:	KEY_Fxx[ 4] <= PRESS;	// F5
		8'h0C:	KEY_Fxx[ 3] <= PRESS;	// F4
		8'h04:	KEY_Fxx[ 2] <= PRESS;	// F3
		8'h06:	KEY_Fxx[ 1] <= PRESS;	// F2
		8'h05:	KEY_Fxx[ 0] <= PRESS;	// F1

		8'h16:	KEY[28] <= PRESS_N;	// 1 !
		8'h1E:	KEY[25] <= PRESS_N;	// 2 @
		8'h26:	KEY[27] <= PRESS_N;	// 3 #
		8'h25:	KEY[29] <= PRESS_N;	// 4 $
		8'h2E:	KEY[24] <= PRESS_N;	// 5 %
		8'h36:	KEY[40] <= PRESS_N;	// 6 ^
		8'h3D:	KEY[45] <= PRESS_N;	// 7 &
//		8'h0D:	KEY[?] <= PRESS_N;	// TAB
		8'h3E:	KEY[43] <= PRESS_N;	// 8 *
		8'h46:	KEY[41] <= PRESS_N;	// 9 (
		8'h45:	KEY[44] <= PRESS_N;	// 0 )
		8'h4E:	KEY[42] <= PRESS_N;	// - _
//		8'h55:	KEY[?] <= PRESS_N;	// = +
		8'h66:	KEY_EX[8] <= PRESS_N;	// backspace
//		8'h0E:	KEY[?] <= PRESS_N;	// ` ~
//		8'h5D:	KEY[?] <= PRESS_N;	// \ |
		8'h49:	KEY[33] <= PRESS_N;	// . >
		8'h4b:	KEY[57] <= PRESS_N;	// L
		8'h44:	KEY[49] <= PRESS_N;	// O
//		8'h11	KEY[?] <= PRESS_N; // line feed (really right ALT (Extended) see below
		8'h5A:	KEY[50] <= PRESS_N;	// CR
//		8'h54:	KEY[?] <= PRESS_N;	// [ {
//		8'h5B:	KEY[?] <= PRESS_N;	// ] }
		8'h52:	KEY[58] <= PRESS_N;	// ' "
		8'h1D:	KEY[ 1] <= PRESS_N;	// W
		8'h24:	KEY[ 3] <= PRESS_N;	// E
		8'h2D:	KEY[ 5] <= PRESS_N;	// R
		8'h2C:	KEY[ 0] <= PRESS_N;	// T
		8'h35:	KEY[48] <= PRESS_N;	// Y
		8'h3C:	KEY[53] <= PRESS_N;	// U
		8'h43:	KEY[51] <= PRESS_N;	// I
		8'h1B:	KEY[ 9] <= PRESS_N;	// S
		8'h23:	KEY[11] <= PRESS_N;	// D
		8'h2B:	KEY[13] <= PRESS_N;	// F
		8'h34:	KEY[ 8] <= PRESS_N;	// G
		8'h33:	KEY[56] <= PRESS_N;	// H
		8'h3B:	KEY[61] <= PRESS_N;	// J
		8'h42:	KEY[59] <= PRESS_N;	// K
		8'h22:	KEY[17] <= PRESS_N;	// X
		8'h21:	KEY[19] <= PRESS_N;	// C
		8'h2a:	KEY[21] <= PRESS_N;	// V
		8'h32:	KEY[16] <= PRESS_N;	// B
		8'h31:	KEY[32] <= PRESS_N;	// N
		8'h3a:	KEY[37] <= PRESS_N;	// M
		8'h41:	KEY[35] <= PRESS_N;	// , <
		8'h15:	KEY[ 4] <= PRESS_N;	// Q
		8'h1C:	KEY[12] <= PRESS_N;	// A
		8'h1A:	KEY[20] <= PRESS_N;	// Z
		8'h29:	KEY[36] <= PRESS_N;	// Space
//		8'h4A:	KEY[?] <= PRESS_N;	// / ?
		8'h4C:	KEY[60] <= PRESS_N;	// ; :
		8'h4D:	KEY[52] <= PRESS_N;	// P
		8'h14:	KEY[10] <= PRESS_N;	// Ctrl either left or right
		8'h12:	KEY[18] <= PRESS_N;	// L-Shift
		8'h59:	KEY_EX[0] <= PRESS_N;	// R-Shift
		8'h11:
		begin
			if(~EXTENDED)
					KEY_EX[1] <= PRESS_N;	// Repeat really left ALT
			else
					KEY_EX[2] <= PRESS_N;	// LF really right ALT
		end
		8'h76:	KEY_EX[3] <= PRESS_N;	// Esc
		8'h75:	KEY_EX[4] <= PRESS_N;	// up
		8'h6B:	KEY_EX[5] <= PRESS_N;	// left
		8'h74:	KEY_EX[6] <= PRESS_N;	// right
		8'h72:	KEY_EX[7] <= PRESS_N;	// down
		endcase
	end
end


`ifdef SIMULATE
initial
	begin
		KB_CLK = 5'b0;
	end
`endif

always @ (posedge CLK50MHZ)				// 50MHz
	KB_CLK <= KB_CLK + 1'b1;			// 50/32 = 1.5625 MHz

ps2_keyboard KEYBOARD(
		.RESET_N(RESET_N),
		.CLK(KB_CLK[4]),
		.PS2_CLK(PS2_KBCLK),
		.PS2_DATA(PS2_KBDAT),
		.RX_SCAN(SCAN),
		.RX_PRESSED(PRESS),
		.RX_EXTENDED(EXTENDED)
);

assign PRESS_N = ~PRESS;


`ifdef AUDIO_WM8731

AUDIO_IF AUD_IF(
	//	Audio Side
	.oAUD_BCLK(AUD_BCLK),
	.oAUD_DACLRCK(AUD_DACLRCK),
	.oAUD_DACDAT(AUD_DACDAT),
	.oAUD_ADCLRCK(AUD_ADCLRCK),
	.iAUD_ADCDAT(AUD_ADCDAT),
	//	Control Signals
	.iSPK_A(SPEAKER_A),
	.iSPK_B(SPEAKER_B),
	.iCASS_OUT(CASS_OUT),
	.oCASS_IN_L(CASS_IN_L),
	.oCASS_IN_R(CASS_IN_R),
	// System
	.iCLK_18_4(AUD_CTRL_CLK),
	.iRST_N(RESET_N)
);

I2C_AV_Config AUD_I2C(
	//	Host Side
	.iCLK(CLK50MHZ),
	.iRST_N(RESET_N),
	//	I2C Side
	.I2C_SCLK(I2C_SCLK),
	.I2C_SDAT(I2C_SDAT)
);

assign	AUD_XCK = AUD_CTRL_CLK;

`endif


`ifdef AUDIO_GPIO
`endif


assign	CASS_OUT		=	{LATCHED_IO_DATA_WR[2], 1'b0};

`ifdef AUDIO_WM8731
assign	CASS_IN			=	CASS_IN_L;
`endif

`ifdef AUDIO_GPIO
assign	CASS_IN			=	1'b0;
`endif


`ifdef GPIO_PIN
//assign	GPIO_0[35:0]	=	36'bz;		//	GPIO Connection 0
//assign	GPIO_1[35:0]	=	36'bz;		//	GPIO Connection 1
`endif

// other

assign LED = {4'b0, CASS_IN, CASS_IN_L, CASS_IN_R, CASS_OUT, A_KEY_PRESSED, CPU_RESET};

endmodule
