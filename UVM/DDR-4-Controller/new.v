/***************************************************************************************************************************
*
*    File Name:  Counter.sv
*      Version:  1.0
*        Model:  Internal Delay counter
*
* Dependencies:  DUT.sv.sv
*				 
*
*  Description:  Acts as a down counter to generate the delay for obeying the timing specifications.
*
* Rev   Author   Date        Changes
* ---------------------------------------------------------------------------------------
* 0.1    SA      02/28/18    FSM design
* 0.42  JMK      08/25/06    Created internal clock using ck and ck_n.

*****************************************************************************************************************************/

module counter (                           //counter Module
	input  logic       clock    ,          //Clock fro the counter from Emulator
	input  logic       reset    ,          //Reset
	input  logic       en,                 //Counter Enable 
	input  logic [31:0] max_count,         //Set the counter to a Maximum Value
	output bit         done     ,          //if done , counter has reached it's max_count value
	output logic [31:0] count              //counter is active when enable signal is on and counts till it reaches max_count
);
//==================================================================================================================================================
//Counter for DDR3 memory Model depending on the max_count value set the counter gets incremented and when count==max_count-1 then the done signal is
//asserted
	always@(posedge clock) begin           //Activates at posedge of Clock
		if((reset) | (count==max_count-1)) //if reset or reaches max count value resets the counter value to 0
			count <= 0;                    //assigning the counter value to zero if above condition is true
		else if(en)                        // if enable is high the counter starts counting it's value at every positive clock edge 
			count <= count+1;              //count gets incremented when enable signal is high and reaches to max count-1
	end

	assign done = (count==max_count-1);    //assigning the done =1 when counter reaches it's maximum value.

  ------------------------------------------------------------------------------------------------------------------------------------------------
  
/***************************************************************************************************************************
*
*    File Name:  DUT.sv
*      Version:  1.0
*        Model:  Memory Controller
*
* Dependencies:  DUT_pkg.sv
*				 ddr3.sv (Memory Model)
*
*  Description:   Memory controller for Micron SDRAM DDR3-800 (Double Data Rate 3)
*
*   Functions :  - Performs following operations
*				   - POWERUP SEQUENCE, ZQ CALIBRATION, MODE REGISTER LOAD
*				   - ACTIVATE , WRITE, READ (Burst mode), PRECHARGE.
*				   - Works according to the timing specification followed by the memory model.
*				   - Timing specs : 6-6-6.

*****************************************************************************************************************************/

//===================================== PACKAGE IMPORT========================================================================
import DDR3MemPkg::* ;


//===================================== MODULE DECLARATION ===================================================================
module DDR3_Controller (
	input logic          i_cpu_ck   ,												// Main system clock 
	input logic          i_cpu_ck_ps,												// 90degree phase shifted clock
	cpu_if.dut_port      cont_if_cpu,												// Interface between CPU-CONTROLLER
	mem_if.dut_port      cont_if_mem												// Interface between MEM-CONTROLLER
);


//===================================== LOCAL VARIABLES=======================================================================
			
    bit    [31:0] max_count                     ;   // variable to assign max count
    bit    [ 0:0] rst_counter                   ;	// Reset counter variable
    bit           t_dqs_flag,t_dqsn_flag        ;   // internal flags for DQS and DQSN strobe
	logic         rw_flag,timer_intr            ;	// Flags for R/W and  Timer Interrupt
	logic         t_flag                        ;	// Timer Flag
	logic         dq_valid                     ;	// DQS valid signal 
	logic  [31:0] v_count                       ;	// Internal counter variable	
	logic  [15:0] wdata              [3:0]      ;	// 16 bit Write data
	logic  [15:0] rdata              [3:0]      ;	// 16 bit read data
	logic  [ 7:0] t_dq_local                    ;	
	logic  [15:0] wdata_local                   ;	// Local variable for write data
	logic  [15:0] rdata_local                   ;	// Local variable for read data
	logic         en                            ;	// Internal enable signeal
	logic  [26:0] s_addr                        ;	// 27 bit address variable
	logic  [63:0] s_data                        ;	// 64 bit data variable
	logic         s_valid_data             ;	// DAta valid during read
	logic  [ 7:0] temp1, temp2                  ;
	logic  [63:0] s_cpu_rd_data                 ;	// internal variable for CPU read operation 
	logic  [63:0] cpu_rd_data                   ;	
	logic         s_cpu_rd_data_valid           ;	// Valid signal for CPU read data
	logic         cpu_rd_data_valid             ;

	
	States        state                         ;   


//============================================================ INSTANTIATIONS============================================================================
// Instantiation of internal counter
	counter i_counter (.clock(i_cpu_ck), .reset(cont_if_cpu.i_cpu_reset), .en(en), .max_count(max_count), .done(timer_intr), .count(v_count));

// Instantiate of Write Burst module
	WriteBurst #(8) i_WriteBurst (.clock(i_cpu_ck_ps), .data(wdata_local), .out(t_dq_local), .valid_in(s_valid_data), .valid_out(dq_valid), .reset(cont_if_cpu.i_cpu_reset));
	
// Instantiation of read burst module
	read_burst #(8) i_ReadBurst (.clock(i_cpu_ck_ps), .data_in(cont_if_mem.dq), .out(rdata_local));

//============================================================ COMBINATIONAL ASSIGNMENTS=================================================================
	assign cont_if_mem.ck   = ~i_cpu_ck;											 	// Internal clock assignmnet 
	assign cont_if_mem.ck_n = i_cpu_ck;


	assign s_valid_data = (state==WBURST) & (v_count>=0);								// set s_valid_data in order to send the burst to memory	(Write operation)

	always_comb cont_if_cpu.o_cpu_data_rdy <= (state==IDLE);							// set ready signal from CPU when in IDLE state

//============================================================ SEQUENTIAL LOGIC===========================================================================
// // Assign internal data and valid signals to CPU read and valid signals	
 	always_ff@(posedge i_cpu_ck)
 		begin
 			cont_if_cpu.o_cpu_rd_data       <= cpu_rd_data;							
			cont_if_cpu.o_cpu_rd_data_valid <= s_cpu_rd_data_valid;					 
		end

// Read Burst operation. Provide 16 bits to the CPU per clock cycle.  
	always_ff @(negedge i_cpu_ck) begin : proc_r_burst
		if(cont_if_cpu.i_cpu_reset)										
			cpu_rd_data <= 0;
		else if(state==RBURST) 															
			unique case (v_count)													    
			3       : cpu_rd_data[63:48] <= rdata_local;
			2       : cpu_rd_data[47:32] <= rdata_local;
			1       : cpu_rd_data[31:16] <= rdata_local;
            0       : cpu_rd_data[15: 0] <= rdata_local;
			default : cpu_rd_data <= 0;
		endcase
	end

//=================================================================== STATE TRANSITION BLOCK=========================================================
	always_ff@(posedge i_cpu_ck) begin
		if(cont_if_cpu.i_cpu_reset)											
			state <= POWERUP;										// state to POWERUP on reset
		else
			unique case(state)
				POWERUP : begin
					if(timer_intr)									// TXPR cycle meet to escape CKE high
						state <= ZQ_CAL;							// State to ZQ_CAL on timer interrupt
				end

				ZQ_CAL : begin
					if(timer_intr)
						state <= CAL_DONE;							// State to CAL_DONE on timer interrupt
				end

				CAL_DONE : begin
					state <= MRLOAD;								// State to MRLOAD on timer interrupt
				end

				MRLOAD : begin
					if(timer_intr)									
						state <= IDLE;								// State to IDLE on timer interrupt
				end

				IDLE : begin
					if(cont_if_cpu.i_cpu_valid)
						state <= ACT;								// State to ACT if CPU valid signal is high
				end

				ACT : begin
					if(timer_intr) begin
						if(rw_flag == 1)						    // Check for Read/Write
							state <= WRITE;							
						else
							state <= READ;
					end
				end

				WRITE : begin
					if(timer_intr)
						state <= WBURST;							//  State to WBURST on timer interrupt
				end

				READ : begin
					if(timer_intr)
						state <= RBURST;							// State to READ BURST on timer interrupt
				end

				WBURST : begin
					if(timer_intr)
						state <= AUTORP;							// State to PRECHARGE on timer interrupt
				end

				RBURST : begin
					if(timer_intr)
						state <= AUTORP;							// State to PRECHARGE on timer interrupt
				end

				AUTORP : begin
					if(timer_intr)
						state <= DONE;
				end

				DONE : begin
					state <= IDLE;
				end

				default : state <= POWERUP;							// State to POWERUP by default


			endcase
	end


//======================================================== OUTPUT BLOCK=============================================================================
// Begin with reseting the controller outputs to deassert condition.
	always_comb begin
		cont_if_mem.rst_n   = 1'b1;							// deassert reset signal
		cont_if_mem.odt     = 1'b1;						    // Set on die terminal signal
		cont_if_mem.ras_n   = 1'b1;							
		cont_if_mem.cas_n   = 1'b1;
		cont_if_mem.cs_n    = 1'b0;
		cont_if_mem.we_n    = 1'b1;
		cont_if_mem.ba      = 'b0;							// set bank address variable to 0
		cont_if_mem.addr    = 'b0;							// Set memory adddress variable to 0
		cont_if_mem.cke     = 'b1;							// Set Clock enable signal
		t_flag              = 'b0;				
		en                  = 'b0;							// set enable signal to 0
		s_cpu_rd_data_valid = 0;							// Set read data valid to 0
		s_cpu_rd_data       = 0;							// Set cpu data to 0
		case(state)
		// In this mode the DDR is powerup at clock cycle = 5 by setting rst_n to high and odt to 0	
		// After 9 clock cycles, odt is set along with performing chip select
			POWERUP : begin
				// RESET
				max_count         = 'd57;					
				cont_if_mem.rst_n = 1'b0;
				cont_if_mem.cke   = 1'b0;
				cont_if_mem.cs_n  = 1'b1;
				cont_if_mem.odt   = 1'b0;
				en                = 1'b1;
				// POWER UP AND CLOCKING DDR CHIP
				if(v_count>='d5) begin
					cont_if_mem.rst_n = 1'b1;
					cont_if_mem.odt   = 1'b0;
				end
				if(v_count>='d9) begin
					cont_if_mem.cke  = 1'b1;
					cont_if_mem.odt  = 1'b1;
					cont_if_mem.cs_n = 1'b0;
					cont_if_mem.odt  = 1'b0;
				end
			end

			// This state involves setting the A10 bit to enable the Auto Precharge Functionality 
			ZQ_CAL : begin
				max_count       = 'd1;
				en              = 1'b1;
				cont_if_mem.odt = 1'b0;
				// ZQ CALIBRATION PRECHARGING ALL THE BANKS
				if(v_count=='d0) begin
					cont_if_mem.we_n = 1'b0;
					cont_if_mem.ba   = 'd0;
					cont_if_mem.addr = 14'b00010000000000;
					cont_if_mem.odt  = 1'b0;
				end
			end

			// counter is set to max count of 4*T_MRD
			// Enable is set .
			// Mode registers are configured after every T_MRD clock cycle.
			MRLOAD : begin
				cont_if_mem.odt = 1'b0;
				max_count       = 4*T_MRD;
				en              = 1'b1;
				if(v_count=='d0) begin						// Mode Register0 with DLL Reset
					cont_if_mem.ras_n = 1'b0;
					cont_if_mem.cas_n = 1'b0;
					cont_if_mem.we_n  = 1'b0;
					cont_if_mem.ba    = 3'b011;				// Config bank 3
					cont_if_mem.addr  = 14'b0;
					cont_if_mem.odt   = 1'b0;
				end
				else if(v_count==T_MRD) begin 				// Extended Mode Register1 with DLL Enable, AL=CL-1
					cont_if_mem.ras_n = 1'b0;
					cont_if_mem.cas_n = 1'b0;
					cont_if_mem.we_n  = 1'b0;
					cont_if_mem.ba    = 3'b010;				// Config bank 2
					cont_if_mem.addr  = 14'b00000000000000;
					cont_if_mem.odt   = 1'b0;
				end
				else if(v_count==2*T_MRD) begin				// Extended Mode Register2 with DCC Disable
					cont_if_mem.ras_n = 1'b0;
					cont_if_mem.cas_n = 1'b0;
					cont_if_mem.we_n  = 1'b0;
					cont_if_mem.ba    = 3'b001;				// Config Bank 1
					cont_if_mem.addr  = 14'b00000000010110;
					cont_if_mem.odt   = 1'b0;
				end
				else if(v_count==3*T_MRD) begin 			// Extended Mode Register3
					cont_if_mem.ras_n = 1'b0;
					cont_if_mem.cas_n = 1'b0;
					cont_if_mem.we_n  = 1'b0;
					cont_if_mem.ba    = 3'b000;				// Config Bank 0
					cont_if_mem.addr  = 14'b00010100011000;
					cont_if_mem.odt   = 1'b0;
				end
			end

			// Reset on die termination
			CAL_DONE : cont_if_mem.odt   = 1'b0;			

			// Set the maximum count to T_RCD
			// During ACT, Bank and Row address are provided 
			// ras is assserted.
			// The controller has to wait for a period of Row-Column Delay
			ACT : begin
				max_count = T_RCD+1;
				en        = 1'b1;
				if(v_count=='d0) begin
					cont_if_mem.ba    = s_addr[12:10];		// 3 Bits for Bank
					cont_if_mem.addr  = s_addr[26:13];		// 14 row address bits
					cont_if_mem.ras_n = 1'b0;				// check if we_n should be asserted
				end
			end

			// 3LSBs are used for byte selec, which is why they are set to 0. 
			// Hence we obtain burst right from the first byte which reduces the delay
			// byte select is configurable (CRITICAL BYTE FIRST).
			READ : begin
				en              = 1'b1;
				max_count       = T_CL + 4;
				cont_if_mem.odt = 1'b0;
				if(v_count=='d0) begin
					cont_if_mem.we_n  = 1'b1;
					cont_if_mem.ba    = s_addr[12:10];			// provide bank address
					cont_if_mem.addr  = {s_addr[9:3],3'b0};		// 
					cont_if_mem.cas_n = 1'b0;
				end
			end

			WRITE : begin
				en        = 1'b1;
				max_count = T_CL-1+3;
				if(v_count=='d0) begin
					cont_if_mem.we_n  = 1'b0;
					cont_if_mem.ba    = s_addr[12:10];
					cont_if_mem.addr  = {s_addr[9:3],3'b0};
					cont_if_mem.cas_n = 1'b0;
				end
			end

			
			RBURST : begin
				en              = 1'b1;									// Set enable
				max_count       = T_RAS-T_CL-T_RCD+1+2;					// set the max count
				cont_if_mem.odt = 1'b0; 
				if(v_count=='d3) begin
					s_cpu_rd_data_valid <= 1;
				end
			end

			// Write burst is performed using the write buffer. the memory provides 64 bits in chuncks of 8 in 4 clock cycles.
			// At every edge these 8 bits are captured and internally alligned to form 16 bits at the next clock edge.
			// After all 64 bits are obtained, the controller provides the entire 64 bits to the controller 
			WBURST : begin
				rst_counter = 'd0;
				en          = 1'b1;
				max_count   = T_RAS-T_CL-T_RCD+2;
				t_dqsn_flag = 'd0;
				wdata[0]    = s_data[15:0];
				wdata[1]    = s_data[31:16];
				wdata[2]    = s_data[47:32];
				wdata[3]    = s_data[63:48];
				t_flag      = (v_count > 0);
				if(v_count=='d0)
					wdata_local = wdata[0];
				else if(v_count=='d1)
					wdata_local = wdata[1];
				else if(v_count=='d2)
					wdata_local = wdata[2];
				else if(v_count=='d3)
					wdata_local = wdata[3];
			end

			// After every row is read, it is closed by performing auto precharge operation.
			// This is achieved by setting the A10 bit to 1 is the address.
			AUTORP : begin
				en        = 1'b1;
				max_count = T_RP;
				if(v_count=='d0) begin
					cont_if_mem.we_n  = 1'b0;
					cont_if_mem.ras_n = 1'b0;
					cont_if_mem.ba    = s_addr[12:10];
					cont_if_mem.addr  = 1<10;
				end
			end
		endcase
	end


//=====================================================TRI STATE LOGIC FOR BIDIRECTIONAL SIGNALS========================================================
// TRISTATING  DQ , DQS
	assign cont_if_mem.dq      = (dq_valid) 	? t_dq_local	:'bz ;							//assign dq to t_dq_local if dq_valid is set
	assign cont_if_mem.dqs     = (s_valid_data) ? i_cpu_ck		:'bz ;
	assign cont_if_mem.dqs_n   = (s_valid_data) ? ~i_cpu_ck		:'bz ;
	assign cont_if_mem.dm_tdqs = (dq_valid) 	? 0 			:'bz ;

// PROC FOR READ WRITE FLAG FROM CPU CMD DURING ACT STATE
	always_ff @(posedge i_cpu_ck or negedge cont_if_cpu.i_cpu_reset) begin : proc_rw
		if((cont_if_cpu.i_cpu_reset) | (state==DONE)) begin
			rw_flag <= 0;
		end else if (cont_if_cpu.i_cpu_valid & cont_if_cpu.i_cpu_cmd)
			rw_flag <= 1;;
	end

// PROC FOR internal address and data assignment during the IDLE state
	always_ff @(posedge i_cpu_ck) begin : proc_addr_data_lacth
		if(cont_if_cpu.i_cpu_reset) begin
			s_addr <= 0;
			s_data <= 0;
		end else if ((cont_if_cpu.i_cpu_valid) & (state==IDLE)) begin
			s_addr <= cont_if_cpu.i_cpu_addr;
			s_data <= cont_if_cpu.i_cpu_wr_data;
		end
	end

 endmodule:DDR3_Controller
endmodule                                  //end of counter module

--------------------------------------------------------------------------------------------------------------------------------------------------

/***************************************************************************************************************************
*
*    File Name:  DUT_pkg.sv
*      Version:  1.0
*        Model:  Package for various parameters
*
* Dependencies:  DUT.sv
*				 
*
*  Description:  Contains generalized parameters for address and Data bits.
*				 Also parameterizes the delays depecding on the memory model specs


*****************************************************************************************************************************/


package DDR3MemPkg;
//==================================================================================================================================================
	// BIT Parameters
	parameter DM_BITS          =       1; // Set this parameter to control how many Data Mask bits are used
	parameter ADDR_BITS        =      14; // MAX Address Bits
	parameter BA_BITS          =       3; // MAX Address Bits
	parameter ROW_BITS         =      14; // Set this parameter to control how many Address bits are used
	parameter COL_BITS         =      10; // Set this parameter to control how many Column bits are used
	parameter DQ_BITS          =       8; // Set this parameter to control how many Data bits are used       **Same as part bit width**
	parameter DQS_BITS         =       1; // Set this parameter to control how many Dqs bits are used
	parameter BURST_L	   	   =	   8; // Burst Length
	parameter ADDR_MCTRL	   =	  32; // Address to the Controller
//==================================================================================================================================================
	// Memory Parameters for Configurations
	parameter T_RAS	 =15;	// From Row Addr to Precharge
	parameter T_RCD	 =6;	// Row to Column Delay
	parameter T_CL	 =6;	// Column to Data Delay
	parameter T_RC	 =21;	// RP to Next RP Delay
	parameter T_BL	 =4;	// Burst Length in cycles
	parameter T_RP	 =6;	// Precharge Time
	parameter T_MRD	 =4;	// Precharge Time
//==================================================================================================================================================
	// FSM States
	typedef enum logic [3:0] { RESET,POWERUP, MRLOAD, ZQ_CAL, CAL_DONE, IDLE, ACT, READ, WRITE, WBURST, RBURST, AUTORP, DONE} States; 
//==================================================================================================================================================
    //defining the states for the FSM States
	States state;

endpackage
-------------------------------------------------------------------------------------------------------------------------------------------------

/***************************************************************************************************************************
*
*    File Name:  interface.sv
*      Version:  1.0
*        Model:  Interface 
*
* Dependencies:  DUT.sv
*				 DDR3MemPkg.sv
*				 
*
*  Description:  contains 2 interfaces. 1. CPU-CONTROLLER    2. CONTROLLER-MEMORY
*				 Contains respective modports in each interface  
*


*****************************************************************************************************************************/

import DDR3MemPkg::* ;                     //Importing the variables for memory parameters and Address bit parameters
//==================================================================================================================================================
//Interface signals between MemController and DRAM memory
interface mem_if(input logic i_cpu_ck);	   //Clock from emulator mem_if Interface
	logic   rst_n;                         //Reset Signal
    logic   ck;                            // complement of CPU Clock
    logic   ck_n;                          //CPU Clock
    logic   cke;                           //Clock_enable from MemController to Memory
    logic   cs_n;                          //Chip Select Signal
    logic   ras_n;                         //RAS Signal row to column signal
    logic   cas_n;                         //CAS Signal column to data delay signal
    logic   we_n;                          //Write or read enable signal
    tri   [1-1:0]   dm_tdqs;
    logic   [BA_BITS-1:0]   ba;            // bank Bits 
    logic   [ADDR_BITS-1:0] addr;          //MAX Address Bits for the address bus
    tri   [DQ_BITS-1:0]   dq;              //data bits from/to memory controller form memory or CPU
    tri   [1-1:0]  dqs;                    //data strobe signal
    tri   [1-1:0]  dqs_n;                  //Checks if data is valid and assigned to complement of Cpu clock
    logic  [1-1:0]  tdqs_n;                //terminating Data strobe signal
    logic   odt;                           //on-die terminating Signal

//======Module port for controller signals===============================================================
	modport contr_sig (
		output ck, ck_n, rst_n, cs_n, cke, ras_n, cas_n, we_n, odt, ba, addr,tdqs_n,
		inout dm_tdqs, dq, dqs, dqs_n
	);


//======Module ports for Memory===========================================================================
	modport mem_sig (
		input ck, ck_n, rst_n, cs_n, cke, ras_n, cas_n, we_n, odt,ba, addr,tdqs_n,
		inout dm_tdqs,dq, dqs, dqs_n
	);


endinterface : mem_if

///////////////////////// Interface for Driver///////////////////////////


//==================================================================================================================================================
//Interface between CPU and Memory Controller
interface mem_intf(input logic i_cpu_ck);
   
  	//logic	     				i_cpu_ck;		// Clock from TB
	logic	     				i_cpu_reset;	// Reset passed to Controller from TB
	logic [ADDR_MCTRL-1:0]		i_cpu_addr;  	// Cpu Addr
	logic 	     				i_cpu_cmd;		// Cpu command RD or WR
	logic [8*DQ_BITS-1:0]		i_cpu_wr_data;	// Cpu Write Data 
	logic 	     				i_cpu_valid;	// Valid is set when passing CPU addr and command
	logic 	     				i_cpu_enable;	// Chip Select
	logic [BURST_L-1:0]  		i_cpu_dm;		// Data Mask - One HOT
	logic [$clog2(BURST_L):0]	i_cpu_burst;	// Define Burst Length - wont be used for now
	logic [8*DQ_BITS-1:0]		o_cpu_rd_data;	// Cpu data Read
	logic	     				o_cpu_data_rdy;	// Cpu data Ready	
	logic 						o_cpu_rd_data_valid; // Signal for valid data sent to CPU   
	

  modport MemController (
		input   i_cpu_ck,                    // Clock from TB
		input 	i_cpu_reset,               //Reset passed to controller from TB
		input 	i_cpu_addr,                //CPU Address to MemController
		input 	i_cpu_cmd,                 //CPU command Read or write
		input 	i_cpu_wr_data,             //CPU write Data
		input 	i_cpu_valid,               //Valid is set when passing CPU Addr and Command
		input 	i_cpu_enable,              //Enable Signal
		input 	i_cpu_dm,                  //Data mask-One Hot
		input 	i_cpu_burst,               //Defining the Burst Lenght
		output  o_cpu_rd_data,             //CPU data Read
		output  o_cpu_data_rdy,	           //CPU data Ready
		output 	o_cpu_rd_data_valid);      //Signal for Valid data sent to CPU

int count;
  
//==================================================================================================================================================
//Reset Function -Reset Condition (Signal to reset the MemController)
task Reset();                                                     //Reset Function -Reset Condition (Signal to reset the MemController)
		@(posedge i_cpu_ck);                                      //At first posedge of Clock from TB
		$display("--------- [DRIVER] Reset Started ---------");   
		i_cpu_reset = 1;                                          //Reset signal is made high
		i_cpu_valid = 0;                                          // Valid signal Set low
		i_cpu_enable= 0;                                          //deselecting the CPU enable signal
		@(posedge i_cpu_ck);                                      //At Second Posedge clk
		i_cpu_reset = 0;                                          //Reset Signal to 0
		i_cpu_enable = 1;                                         //Enabling the CPU Enable Signal
		$display("--------- [DRIVER] Reset Ended---------");
endtask
                                                                                 //Signals needed to be asserted while in write operation
//==================================================================================================================================================
//sending 64  bits of data from CPU to MemController and the required signals need to be asserted in write operation
task Write(logic [ADDR_MCTRL-1:0] address, logic [8*DQ_BITS-1:0] write_data);    //passing Address and write data to memory Controller
	@(posedge i_cpu_ck);                                                         //At Posedge of Clock
		wait (o_cpu_data_rdy);                                                   //wait till cpu data is ready
		@(posedge i_cpu_ck);                                                     //if cpu data ready in the coming posedge of clock
		i_cpu_valid=1'b1;                                                        //Address valid to 1 ,address valid
		i_cpu_cmd=1'b1;                                                          //write command to MemController
		if (i_cpu_valid && i_cpu_cmd) begin									     //if valid and write enable are both high 				
				i_cpu_addr=address;                                              //address is stored in i_cpu_addr
				i_cpu_wr_data=write_data;                                        //cpu write data i_cpu_wr_data
		end
		@(posedge i_cpu_ck);                                                     //in the next clock cycle
		i_cpu_valid=0;                                                           //making the valid signal to go low
endtask                                                                          //end write task

                                                                                     //Signals needed to be asserted while in read operation
//==================================================================================================================================================
//Sendin the data from  MemController to CPU and the required signals need to be asserted in read operation
task Read(logic [ADDR_MCTRL-1:0] address, output logic [8*DQ_BITS-1:0] read_data );  //passing Address as Input and taking out data from memory Controller
	@(posedge i_cpu_ck); 															 //At Posedge of Clock
		wait(o_cpu_data_rdy);                                                        //wait till cpu data is ready
		@(posedge i_cpu_ck);                                                         //if cpu data ready in the coming posedge of clock
		i_cpu_valid=1'b1;  															 //Address valid to 1 ,address valid
		i_cpu_cmd=1'b0;																 //Read command to MemController
		if (i_cpu_valid && ~i_cpu_cmd) begin										 //checking if read operation from cpu is valid
				i_cpu_addr=address;                                                  //address is stored in i_cpu_addr
				@(posedge i_cpu_ck);                                                 //in the next clock cycle
				i_cpu_valid=0;                                                       //making the valid signal to go low
				wait(o_cpu_rd_data_valid);                                           //wait till valid signal from MemController
				//@(posedge i_cpu_ck);
				read_data = o_cpu_rd_data;                                           //Capturing the data from CPU
				end
		
endtask                                                                              //end read operation
                                                                                     //Run -Operation
//==================================================================================================================================================
//checks the address and data coming from cpu depending on that it does the required operations R/W opearions if the memoery controller is in 
//IDLE State 
task run(logic valid, logic cmd, logic [ADDR_MCTRL-1:0] address,                     //passing Valid,Command(R/W),Address,Write Data
         logic [8*DQ_BITS-1:0] wr_data, output logic [8*DQ_BITS-1:0] rd_data);       //output data from MemController if read 
		$display("count=%d", count++);
		@(posedge i_cpu_ck);                                                          //in the posedge of clock cycle
		wait (o_cpu_data_rdy);                                                        //wait till cpu data is ready
		@(posedge i_cpu_ck);                                                          //in the posedge of clock cycle
		if(valid) begin                                                               //if valid is High
		@(posedge i_cpu_ck);                                                          //in the posedge of clock cycle
		i_cpu_valid=valid;                                                            //assign valid from CPU to Interface valid 
		i_cpu_cmd=cmd;                                                                //assign valid from CPU to Interface valid 
		if (valid && cmd) begin										                  //if valid and write enable are both high then write operation				
				i_cpu_addr=address;                                                   //address is sent to i_cpu_addr (MemController)
				i_cpu_wr_data=wr_data;                                                //write data to interface(MemController)
				@(posedge i_cpu_ck);                                                  //in the posedge of clock cycle
				i_cpu_valid=0;                                                        //making the valid signal to go low
				end
		
		if (valid && ~cmd) begin												      //if valid is high and write enable is low then read operation										// Read
				i_cpu_addr=address;                                                   //address is sent to i_cpu_addr (MemController)
				@(posedge i_cpu_ck);                                                  //in the posedge of clock cycle
				i_cpu_valid=0;                                                        //making the valid signal to go low
				@(posedge o_cpu_rd_data_valid);                                       //at posedge of data valid
				//@(posedge i_cpu_ck);
				rd_data = o_cpu_rd_data;                                              //outof data from MemController to cpu
				end
		end
endtask                                                                               //end the task for Run Operation

endinterface : mem_intf                                                               //end mem_intf

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
/***************************************************************************************************************************
*
*    File Name:  Data_O_Burst.sv
*      Version:  1.0
*        Model:  Read Burst
*
* Dependencies: DUT.sv 
*				 
*
*  Description:  Receives 8 bits per clock edge from the memory.
*				 After one clock cycle, concatenates the 8 bits to for a 16 bit output.
*
* Rev   Author   Date        Changes
* ---------------------------------------------------------------------------------------
* 0.1    SA      02/28/18    FSM design
* 0.42  JMK      08/25/06    Created internal clock using ck and ck_n.

*****************************************************************************************************************************/


module read_burst #(parameter BW = 8)(input logic clock,   //Read Burst from Memory to Memory Controller
				  input logic [BW-1:0] data_in,            // input Data to  dram Buffers
				  output logic [BW*2-1:0] out);            //  output read data

//==================================================================================================================================================
// Local variables
logic [BW-1:0] temp1;								       // Temperory variable to store output data
logic [BW-1:0] temp2;
logic valid_out;								           // Data assigned to CPU only if valid_out is set

//==================================================================================================================================================
//DRAM buffer , at every poesedge and negedge of clk we capture 8 bits of data and send it 16 bits of Data in every clock cycles to Memory Controller
always @ (posedge clock) begin            //in the posedge of the clock we store first 8 bits of data in temp2
	temp2 <= data_in;
end

always @ (negedge clock) begin            //in the negedge of the clock we store next 8 bits of data in temp2
	temp1 <= data_in;
end

always_ff @ (negedge clock) begin         //in the posedge of the clock we send the 16 bits of data
	out <= {temp2, temp1};
end

endmodule // read_burst
-------------------------------------------------------------------------------------------------------------------------------------------------------

/***************************************************************************************************************************
*
*    File Name:  Burst.sv
*      Version:  1.0
*        Model:  Write burst
*
* Dependencies:  DUT_pkg.sv
*				 
*
*  Description:  Performs the burst operation by writing by taking 16 bits per clock cycle as inputs
*				 Sends 8 bits of data at every clock edge to the memory.
*
*

*****************************************************************************************************************************/

module WriteBurst #(parameter BW=8)(      //Write Burst 
	input  logic        clock    ,        //Input clock
	input  logic        reset    ,        //Input Reset 
	input  logic [2*BW-1:0] data     ,    //Input data to Memory Controller
	input  logic        valid_in ,        //Input Reset
	output logic [BW-1:0] out      ,      // Data out  in 8 bits
	output logic        valid_out         //Valid Data out Signal
);
	logic [BW-1:0] temp1;                 //Temperory variable to store output data
	logic [BW-1:0] temp2;                 //Temperory variable to store output data
	logic valid_out1,valid_out2;          //Valid Signal for temp1,temp2 data is valid 

	
//==================================================================================================================================================
	assign out = (clock) ? temp2 : temp1;           //Sending out data as it is centre aligned for write

	assign valid_out = (valid_out1 & valid_out2);   //Valid is asserted high if both the temp1 and temp2 valid signals are high
	
//==================================================================================================================================================
//keep the track of data to be send to memory at every posedge and negdedge by setting the valid signal 
	always_ff @(negedge clock) begin : proc_valid1  
		if(reset) begin                             //In negedge of Clock if reset is assigned to 0
			valid_out1 <= 0;
		end else begin                              
			valid_out1 <= valid_in;                 //valid signal from Memory Controller
		end
	end
	always_ff @(posedge clock) begin : proc_valid2  //Valid signal for data
		if(reset) begin                             //In negedge of Clock if reset valid is assigned to 0
			valid_out2 <= 0;
		end else begin
			valid_out2 <= valid_in;                 //valid signal from Memory Controller
		end
	end
	
//==================================================================================================================================================
//send the data to Memory in chunks of 8 bits in every posedge and negedge of Clock Cycle
	always @ (posedge clock) begin            //Capturing LSB of 8 bits in posedge of clock
		if(valid_in)
			temp1 <= data[BW-1:0];
	end
	always @ (negedge clock) begin            //Capturing MSB of 8 bits in negedge of clock
		if(valid_in)
			temp2 <= data[2*BW-1:BW];
	end

//==================================================================================================================================================
endmodule:WriteBurst
-------------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        cfs_cpu_agent.sv
// Author:      Mohamed Ehab
// Date:        2025-03-24
// Description: CPU agent class.
///////////////////////////////////////////////////////////////////////////////

`ifndef CFS_CPU_AGENT_SV
  `define CFS_CPU_AGENT_SV

  class ddr3_cpu_agent extends uvm_agent;
    `uvm_component_utils(ddr3_cpu_agent)
    
    //Agent configuration handler
    ddr3_cpu_config agent_config;
    
    //Driver handler
    ddr3_cpu_driver driver;
    
    //Sequencer handler
    ddr3_cpu_sequencer sequencer;
    
    //Monitor handler
    ddr3_cpu_monitor monitor;
    
    // Pointer to Virtual CPU Interface 
    cpu_vif vif;

    // Constructor  
    function new(string name = "ddr3_cpu_agent" ,uvm_component parent);
      super.new(name,parent);
    endfunction :new
    
    // Build Phase 
    virtual function void build_phase(uvm_phase phase);
      super.build_phase(phase);     
      
      agent_config = ddr3_cpu_config::type_id::create("agent_config", this);    
      monitor = ddr3_cpu_monitor::type_id::create("monitor", this); 
      
      if(agent_config.get_active_passive() == UVM_ACTIVE) begin
        driver    = ddr3_cpu_driver::type_id::create("driver", this);
        sequencer = ddr3_cpu_sequencer::type_id::create("sequencer", this);
      end
      
    endfunction
    
    
    // Connect Phase 
    virtual function void connect_phase(uvm_phase phase); 
      super.connect_phase(phase);
    
      if(!uvm_config_db #(virtual cpu_if)::get(this,"" ,"cpu_if"   ,vif)) begin
        `uvm_fatal(get_type_name(),"failed to get virtual interface inside cpu agent class")
      end
      else begin
        agent_config.set_vif(vif);
      end
      
      monitor.agent_config = agent_config;
      
      if(agent_config.get_active_passive() == UVM_ACTIVE) begin
        driver.seq_item_port.connect(sequencer.seq_item_export);        
        driver.agent_config = agent_config;
      end
      
    endfunction
    
  endclass : ddr3_cpu_agent

`endif
------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_base_item.sv
// Author:      Mohamed Ehab
// Date:        2025-3-22
// Description: CPU agent base sequence item.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_BASE_ITEM_SV
  `define DDR3_CPU_BASE_ITEM_SV

  class ddr3_cpu_base_item extends uvm_sequence_item;   
    `uvm_object_utils(ddr3_cpu_base_item)
    
    function new(string name = "");
      super.new(name);
    endfunction
    
    
  endclass : ddr3_cpu_base_item

`endif
---------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_base_sequence.sv
// Author:      Mohamed Ehab
// Date:        2023-03-25
// Description: CPU base sequence class.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_BASE_SEQUENCE_SV
  `define DDR3_CPU_BASE_SEQUENCE_SV

class ddr3_cpu_base_sequence extends uvm_sequence#(ddr3_cpu_drv_item);
    `uvm_object_utils(ddr3_cpu_base_sequence)
    
    // Constructor  
    function new(string name = "ddr3_cpu_base_sequence");
      super.new(name);
    endfunction :new

  endclass : ddr3_cpu_base_sequence

`endif
------------------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_config.sv
// Author:      Mohamed Ehab
// Date:        2025-03-23
// Description: CPU agent configurations.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_CONFIG_SV
  `define DDR3_CPU_CONFIG_SV

  class ddr3_cpu_config extends uvm_component; 
    `uvm_component_utils(ddr3_cpu_config)
    
    //Virtual interface
    local cpu_vif vif;
    
    //Active/Passive control
    local uvm_active_passive_enum active_passive;   
    
    // Constructor
    function new(string name = "ddr3_cpu_config" ,uvm_component parent);
      super.new(name,parent);
      active_passive = UVM_ACTIVE;
    endfunction : new
    
    
    
    //Getter for the CPU agent virtual interface
    virtual function cpu_vif get_vif();
      return vif;
    endfunction
    
    //Setter for the CPU agent virtual interface
    virtual function void set_vif(cpu_vif value);
      if(vif == null) begin
        vif = value;
      end
      else begin
        `uvm_fatal("ALGORITHM_ISSUE", "Trying to set the CPU virtual interface more than once")
      end
    endfunction
    
    
    
    //Getter for the CPU agent Active/Passive control
    virtual function uvm_active_passive_enum get_active_passive();
      return active_passive;
    endfunction
    
    //Setter for the CPU agent Active/Passive control
    virtual function void set_active_passive(uvm_active_passive_enum value);
      active_passive = value;
    endfunction
    
    
    //Start of Simulation Phase 
    virtual function void start_of_simulation_phase(uvm_phase phase);
      super.start_of_simulation_phase(phase);
      
      if(get_vif() == null) begin
        `uvm_fatal("ALGORITHM_ISSUE", "The CPU virtual interface is not configured at \"Start of simulation\" phase")
      end
      else begin
        `uvm_info("CPU_CONFIG", "The CPU virtual interface is configured at \"Start of simulation\" phase", UVM_DEBUG)
      end
    endfunction
    
    
  endclass : ddr3_cpu_config

`endif
--------------------------------------------------------------------------------------------------------------------
--///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_driver.sv
// Author:      Mohamed Ehab
// Date:        2025-03-23
// Description: CPU agent driver.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_DRIVER_SV
  `define DDR3_CPU_DRIVER_SV

class ddr3_cpu_driver extends uvm_driver #(ddr3_cpu_drv_item);
    `uvm_component_utils(ddr3_cpu_driver)
    
  
    // Sequence item Declaration  
    ddr3_cpu_drv_item item ;
  
    // Pointer to agent configuration
    ddr3_cpu_config agent_config;
  
    // Pointer to Virtual CPU Interface 
    cpu_vif vif;
  
    // Constructor  
    function new(string name = "ddr3_cpu_driver" ,uvm_component parent);
      super.new(name,parent);
    endfunction :new
    
                  
    // Run Phase 
    task run_phase (uvm_phase phase);   
      super.run_phase(phase);
      forever begin    
        item = ddr3_cpu_drv_item::type_id::create("item");   
        seq_item_port.get_next_item(item) ;
        drive_transaction(item) ;
        seq_item_port.item_done() ;    
      end   
    endtask : run_phase 
  
  
    // Drive Transaction task 
    virtual task drive_transaction (ddr3_cpu_drv_item item) ; 
      
      vif = agent_config.get_vif();
      
     @(posedge vif.i_cpu_ck);
      
      vif.i_cpu_reset <= item.cpu_reset;
      
    //  Check Reset
     if(item.cpu_reset) begin 
        vif.i_cpu_addr    <= 0;
        vif.i_cpu_cmd     <= 0;
        vif.i_cpu_wr_data <= 0;
        vif.i_cpu_valid   <= 0;
        vif.i_cpu_enable  <= 0;
        vif.i_cpu_dm      <= 0;
        vif.i_cpu_burst   <= 0;
     end
     else begin 
        
         while(vif.o_cpu_data_rdy !== 1) begin
           @(posedge vif.i_cpu_ck);
         end
          
       //   Drive Command
          vif.i_cpu_valid <= item.cpu_valid; 
          vif.i_cpu_cmd   <= item.cpu_cmd;
          vif.i_cpu_addr  <= item.cpu_addr ;          
       if (item.cpu_cmd)
           vif.i_cpu_wr_data <= item.cpu_wr_data;
        
       @(posedge vif.i_cpu_ck); 
                   
        //  Reset signals after transaction
          vif.i_cpu_addr    <= 0;
          vif.i_cpu_cmd     <= 0;
          vif.i_cpu_wr_data <= 0;
          vif.i_cpu_valid   <= 0;
        
     end
    
    endtask : drive_transaction 
    
  endclass : ddr3_cpu_driver

`endif
-----------------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_drv_item.sv
// Author:      Mohamed Ehab
// Date:        2025-03-22
// Description: CPU agent driver sequence item.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_DRV_ITEM_SV
  `define DDR3_CPU_DRV_ITEM_SV

  class ddr3_cpu_drv_item extends ddr3_cpu_base_item;
    
    // Constructor 
    function new (string name = "ddr3_cpu_drv_item" );
      super.new(name);
    endfunction
    
    // Driven Signals
    rand bit	     		       	cpu_reset;	     
    rand bit [ADDR_MCTRL-1:0]		cpu_addr;  	     
	rand bit 	     				cpu_cmd;		     
    rand bit [8*DQ_BITS-1:0]		cpu_wr_data;	     
	rand bit 	     				cpu_valid;
    
    // Functions Automation (copy, compare and print)
    `uvm_object_utils_begin(ddr3_cpu_drv_item)
    `uvm_field_int (cpu_reset    , UVM_DEFAULT)
    `uvm_field_int (cpu_addr     , UVM_DEFAULT)
    `uvm_field_int (cpu_cmd      , UVM_DEFAULT)
    `uvm_field_int (cpu_wr_data  , UVM_DEFAULT)
    `uvm_field_int (cpu_valid    , UVM_DEFAULT)
    `uvm_object_utils_end
    
    // Basic Constraints 
    constraint cntr_reset      {cpu_reset dist {0:= 10000 , 1:= 1} ; }
    
  endclass : ddr3_cpu_drv_item

`endif
--------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_if.sv
// Author:      Mohamed Ehab
// Date:        2025-3-22
// Description: CPU agent interface.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_IF_SV
  `define DDR3_CPU_IF_SV

   import DDR3MemPkg::* ;

  //Interface between CPU and Memory Controller
  interface cpu_if(input logic i_cpu_ck);
     
	logic	     				i_cpu_reset;	     // Reset passed to Controller from TB
    logic [ADDR_MCTRL-1:0]		i_cpu_addr;  	     // Cpu Addr
	logic 	     				i_cpu_cmd;		     // Cpu command RD or WR
    logic [8*DQ_BITS-1:0]		i_cpu_wr_data;	     // Cpu Write Data 
	logic 	     				i_cpu_valid;	     // Valid is set when passing CPU addr and command
	logic 	     				i_cpu_enable;	     // Chip Select
    logic [BURST_L-1:0]  		i_cpu_dm;		     // Data Mask - One HOT
    logic [$clog2(BURST_L):0]	i_cpu_burst;	     // Define Burst Length - wont be used for now
    logic [8*DQ_BITS-1:0]		o_cpu_rd_data;	     // Cpu data Read
	logic	     				o_cpu_data_rdy;	     // Cpu data Ready	
	logic 						o_cpu_rd_data_valid; // Signal for valid data sent to CPU   
	

    modport dut_port (
		input   i_cpu_ck,                  // Clock from TB
		input 	i_cpu_reset,               //Reset passed to controller from TB
		input 	i_cpu_addr,                //CPU Address to MemController
		input 	i_cpu_cmd,                 //CPU command Read or write
		input 	i_cpu_wr_data,             //CPU write Data
		input 	i_cpu_valid,               //Valid is set when passing CPU Addr and Command
		input 	i_cpu_enable,              //Enable Signal
		input 	i_cpu_dm,                  //Data mask-One Hot
		input 	i_cpu_burst,               //Defining the Burst Lenght
		output  o_cpu_rd_data,             //CPU data Read
		output  o_cpu_data_rdy,	           //CPU data Ready
		output 	o_cpu_rd_data_valid);      //Signal for Valid data sent to CPU

    
  endinterface : cpu_if

`endif
--------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_drv_item.sv
// Author:      Mohamed Ehab
// Date:        2025-03-22
// Description: CPU agent monitor sequence item.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_MON_ITEM_SV
  `define DDR3_CPU_MON_ITEM_SV

  class ddr3_cpu_mon_item extends ddr3_cpu_base_item;
    
    // Constructor 
    function new (string name = "ddr3_cpu_mon_item" );
      super.new(name);
    endfunction
    
    // Monitored Signals
    bit	     		         		cpu_reset;	     
    rand bit [ADDR_MCTRL-1:0]		cpu_addr;  	     
	rand bit 	     				cpu_cmd;		     
    rand bit [8*DQ_BITS-1:0]		cpu_wr_data;	     
	rand bit 	     				cpu_valid;
    logic    [8*DQ_BITS-1:0]	    cpu_rd_data;	     
	logic	     			    	cpu_data_rdy;	    
	logic 						    cpu_rd_data_valid; 
    
    // Functions Automation (copy, compare and print)
    `uvm_object_utils_begin(ddr3_cpu_mon_item)
    `uvm_field_int (cpu_reset            , UVM_DEFAULT)
    `uvm_field_int (cpu_addr             , UVM_DEFAULT)
    `uvm_field_int (cpu_cmd              , UVM_DEFAULT)
    `uvm_field_int (cpu_wr_data          , UVM_DEFAULT)
    `uvm_field_int (cpu_valid            , UVM_DEFAULT)
    `uvm_field_int (cpu_rd_data          , UVM_DEFAULT)
    `uvm_field_int (cpu_data_rdy         , UVM_DEFAULT)
    `uvm_field_int (cpu_rd_data_valid    , UVM_DEFAULT)
    `uvm_object_utils_end
    
    
    
  endclass : ddr3_cpu_mon_item

`endif
-----------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_monitor.sv
// Author:      Mohamed Ehab
// Date:        2025-03-23
// Description: CPU agent monitor.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_MONITOR_SV
  `define DDR3_CPU_MONITOR_SV

class ddr3_cpu_monitor extends uvm_driver #(ddr3_cpu_mon_item);
    `uvm_component_utils(ddr3_cpu_monitor)
    
    // Sequence item Declaration  
    ddr3_cpu_mon_item item ;
  
    // Analysis port
    uvm_analysis_port #(ddr3_cpu_mon_item) cpu_mon_ap;
  
    // Pointer to agent configuration
    ddr3_cpu_config agent_config;
  
    // Pointer to Vertual CPU Interface 
    cpu_vif vif;
  
    // Counter for READ respond 
    int read_count = T_RCD+T_CL+10 ;
  
    // Constructor  
    function new(string name = "ddr3_cpu_monitor" ,uvm_component parent);
      super.new(name,parent);
    endfunction :new
  
    // Build Phase 
    function void build_phase(uvm_phase phase);    
      super.build_phase(phase); 
      cpu_mon_ap  = new("cpu_mon_ap",this);   
    endfunction : build_phase 
    
                  
    
    // Run Phase 
    task  run_phase(uvm_phase phase);
      super.run_phase(phase);   
      forever begin
        collect_transaction(item);    
      end     
    endtask : run_phase 
  
  
    // Collect Transaction task 
    virtual task collect_transaction (ddr3_cpu_mon_item item) ; 
      
      vif = agent_config.get_vif();
      
     while(vif.i_cpu_valid !== 1)  begin 
       @(posedge vif.i_cpu_ck);
     end
      
      
     // Sample command
      item.cpu_valid    = vif.i_cpu_valid;
      item.cpu_cmd      = vif.i_cpu_cmd;
      item.cpu_addr     = vif.i_cpu_addr;
      if(item.cpu_cmd)
        item.cpu_wr_data  = vif.i_cpu_wr_data;
      
     // Sample output
      item.cpu_data_rdy = vif.o_cpu_data_rdy;
      
     if(item.cpu_cmd === 0) begin 
        repeat(read_count) @(posedge vif.i_cpu_ck);
        item.cpu_rd_data         = vif.o_cpu_rd_data;
        item.cpu_rd_data_valid   = vif.o_cpu_rd_data_valid;
     end
      
      // Send the transaction through Analysis port  
      cpu_mon_ap.write(item);
      
    endtask : collect_transaction 
    
  endclass : ddr3_cpu_monitor

`endif
--------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_pkg.sv
// Author:      Mohamed Ehab
// Date:        2025-3-22
// Description: CPU Agent package.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_CPU_PKG_SV
  `define DDR3_CPU_PKG_SV

  `include "uvm_macros.svh"
  `include "ddr3_cpu_if.sv"

  package ddr3_cpu_pkg;
    import uvm_pkg::*;
    import DDR3MemPkg::* ;
    
     //Virtual interface type
     typedef virtual cpu_if cpu_vif;

    `include "ddr3_cpu_base_item.sv"
    `include "ddr3_cpu_drv_item.sv"
    `include "ddr3_cpu_mon_item.sv"
    `include "ddr3_cpu_config.sv"
    `include "ddr3_cpu_driver.sv"
    `include "ddr3_cpu_monitor.sv"
    `include "ddr3_cpu_sequencer.sv"
    `include "ddr3_cpu_agent.sv"
    `include "ddr3_cpu_base_sequence.sv"
    `include "ddr3_cpu_write_seq.sv"
    `include "ddr3_cpu_read_seq.sv"

  endpackage : ddr3_cpu_pkg

`endif
------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_read_sequence.sv
// Author:      Mohamed Ehab
// Date:        2023-03-25
// Description: CPU read sequence class.
///////////////////////////////////////////////////////////////////////////////
 
`ifndef DDR3_CPU_READ_SEQUENCE_SV
  `define DDR3_CPU_READ_SEQUENCE_SV

class ddr3_cpu_read_seq extends ddr3_cpu_base_sequence ;
  `uvm_object_utils(ddr3_cpu_read_seq)
 
  // Handler for drive item
  ddr3_cpu_drv_item item;
  
  // Constructor
  function  new(string name = "ddr3_cpu_write_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    item = ddr3_cpu_drv_item::type_id::create("item") ;
    
    // Reset transaction
    start_item(item) ; 
    assert(item.randomize() with {item.cpu_reset == 1;})           
    else
      `uvm_error(get_type_name(),"randomization failed in ddr3_cpu_write_seq during reset transaction")  
      finish_item(item);
    
    // Write transaction 
    start_item(item) ; 
    assert(item.randomize() with {
           item.cpu_reset == 0 ;
           item.cpu_cmd   == 0 ;})           
    else
      `uvm_error(get_type_name(),"randomization failed in ddr3_cpu_write_seq during reset transaction")  
      finish_item(item);
    
  endtask
  
  
endclass: ddr3_cpu_read_seq

`endif
--------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_sequencer.sv
// Author:      Mohamed Ehab
// Date:        2025-03-23
// Description: CPU agnet sequencer.
///////////////////////////////////////////////////////////////////////////////

class ddr3_cpu_sequencer extends uvm_sequencer #(ddr3_cpu_drv_item);
  `uvm_component_utils(ddr3_cpu_sequencer)
  
  // Constructor
  function new(string name = "ddr3_cpu_sequencer" ,uvm_component parent);
    super.new(name,parent);
  endfunction :new
  
endclass : ddr3_cpu_sequencer 
------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_cpu_write_sequence.sv
// Author:      Mohamed Ehab
// Date:        2023-03-25
// Description: CPU write sequence class.
///////////////////////////////////////////////////////////////////////////////
 
`ifndef DDR3_CPU_WRITE_SEQUENCE_SV
  `define DDR3_CPU_WRITE_SEQUENCE_SV

class ddr3_cpu_write_seq extends ddr3_cpu_base_sequence ;
  `uvm_object_utils(ddr3_cpu_write_seq)
 
  // Handler for drive item
  ddr3_cpu_drv_item item;
  
  // Constructor
  function  new(string name = "ddr3_cpu_write_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    item = ddr3_cpu_drv_item::type_id::create("item") ;
    
    // Reset transaction
    start_item(item) ; 
    assert(item.randomize() with {
           item.cpu_reset == 1;})           
    else
      `uvm_error(get_type_name(),"randomization failed in ddr3_cpu_write_seq during reset transaction")  
      finish_item(item);
    
    // Write transaction 
    start_item(item) ; 
    assert(item.randomize() with {
           item.cpu_reset   == 0 ;
           item.cpu_cmd     == 1 ;
           item.cpu_valid   == 1 ;
           item.cpu_wr_data == 'hffff;})           
    else
      `uvm_error(get_type_name(),"randomization failed in ddr3_cpu_write_seq during reset transaction")  
      finish_item(item);
    
  endtask
  
  
endclass: ddr3_cpu_write_seq

`endif
-------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        cfs_cpu_agent.sv
// Author:      Mohamed Ehab
// Date:        2025-03-24
// Description: Environment class.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_ENV_SV
  `define DDR3_ENV_SV

  class ddr3_env extends uvm_env;
    `uvm_component_utils(ddr3_env)
    
    // CPU agent handler
    ddr3_cpu_agent cpu_agent;
  
    // Constructor  
    function new(string name = "ddr3_env" ,uvm_component parent);
      super.new(name,parent);
    endfunction :new
    
    // Build Phase
    virtual function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      
      cpu_agent = ddr3_cpu_agent::type_id::create("cpu_agent", this);
      
    endfunction 
    
  endclass : ddr3_env

`endif
-------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_pkg.sv
// Author:      Mohamed Ehab
// Date:        2025-03-24
// Description: Environment package.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_ENV_PKG_SV
  `define DDR3_ENV_PKG_SV

  `include "uvm_macros.svh"
  `include "ddr3_cpu_pkg.sv"
  `include "ddr3_mem_pkg.sv"

  package ddr3_env_pkg;
    import uvm_pkg::*;
    import ddr3_cpu_pkg::*;

    `include "ddr3_env.sv"
  endpackage : ddr3_env_pkg

`endif
---------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_mem_if.sv
// Author:      Mohamed Ehab
// Date:        2025-3-22
// Description: memory interface.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_MEM_IF_SV
  `define DDR3_MEM_IF_SV

   import DDR3MemPkg::* ;
  
  //Interface signals between MemController and DRAM memory
  interface mem_if(input logic i_cpu_ck);	   
    
	logic   rst_n;                    //Reset Signal
    logic   ck;                       // complement of CPU Clock
    logic   ck_n;                     //CPU Clock
    logic   cke;                      //Clock_enable from MemController to Memory
    logic   cs_n;                     //Chip Select Signal
    logic   ras_n;                    //RAS Signal row to column signal
    logic   cas_n;                    //CAS Signal column to data delay signal
    logic   we_n;                     //Write or read enable signal
    logic   odt;                      //on-die terminating Signal
    logic   tdqs_n;                   //terminating Data strobe signal
    logic   [BA_BITS-1:0]   ba;       // bank Bits 
    logic   [ADDR_BITS-1:0] addr;     //MAX Address Bits for the address bus
    tri     [DQ_BITS-1:0]   dq;       //data bits from/to memory controller form memory or CPU
    tri     dm_tdqs;
    tri     dqs;                      //data strobe signal
    tri     dqs_n;                    //Checks if data is valid and assigned to complement of Cpu clock
    

	modport dut_port (
		output ck, ck_n, rst_n, cs_n, cke, ras_n, cas_n, we_n, odt, ba, addr,tdqs_n,
		inout dm_tdqs, dq, dqs, dqs_n
	);

endinterface : mem_if
		

`endif
-------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_mem_pkg.sv
// Author:      Mohamed Ehab
// Date:        2025-3-25
// Description: Memory Agent package.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_MEM_PKG_SV
  `define DDR3_MEM_PKG_SV

  `include "uvm_macros.svh"
  `include "ddr3_mem_if.sv"

  package ddr3_mem_pkg;
    import uvm_pkg::*;

//     `include "ddr3_cpu_base_item.sv"
//     `include "ddr3_cpu_drv_item.sv"
//     `include "ddr3_cpu_mon_item.sv"
//     `include "ddr3_cpu_config.sv"
//     `include "ddr3_cpu_driver.sv"
//     `include "ddr3_cpu_monitor.sv"
//     `include "ddr3_cpu_sequencer.sv"
//     `include "ddr3_cpu_agent.sv"
//     `include "ddr3_cpu_base_sequence.sv"
//     `include "ddr3_cpu_write_seq.sv"
//     `include "ddr3_cpu_read_seq.sv"

  endpackage : ddr3_mem_pkg

`endif
----------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        testbench.sv
// Author:      Mohamed Ehab
// Date:        2025-03-24
// Description: Testbench module. It contains the instance of the DUT and the 
//              logic to start the UVM test and UVM phases.
///////////////////////////////////////////////////////////////////////////////

`timescale 1ps/1ps
`include "ddr3_test_pkg.sv"


module top();
  
  import uvm_pkg::*;
  import ddr3_test_pkg::*;

  parameter tck = 2500/2;
  parameter ps = 2500/4;
  bit i_cpu_ck;
  bit i_cpu_ck_ps;
  

   // Clock generator
   always i_cpu_ck = #4 ~i_cpu_ck;
   always i_cpu_ck_ps = #2 i_cpu_ck;
	

   // Interface Instance 
    cpu_if cpu_intf(i_cpu_ck);   // Instance of CPU-CONTR Interface
    mem_if mem_intf(i_cpu_ck);   // Instance of CONTR-MEM Interface
				
	

   // Controller Instance
	DDR3_Controller	DDR3(
						  .i_cpu_ck(i_cpu_ck),				// System Clock
					      .i_cpu_ck_ps(i_cpu_ck_ps),
                          .cont_if_cpu(cpu_intf.dut_port),		// CPU-CONTR ports
                          .cont_if_mem(mem_intf.dut_port));			// CONTR-MEM ports	


   // Set the virtual interface handles to the config_db 
  initial begin     
    uvm_config_db # (virtual cpu_if)::set(null,"uvm_test_top.env.cpu_agent","cpu_if",cpu_intf);   
    
    run_test("ddr3_write_test");
  end
  
  
  initial 
  begin
    // Required to dump signals to EPWave
    $dumpfile("dump.vcd");
    $dumpvars(0);
  end


endmodule : top	
---------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_base_test.sv
// Author:      Mohamed Ehab
// Date:        2025-03-25
// Description: Basic test class. It creates the instance of the environment.
//              This class should be the parent of all the tests used in the
//              verification of the DDR3 Controller.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_BASE_TEST_SV
  `define DDR3_BASE_TESTE_SV

  class ddr3_base_test extends uvm_test;
    `uvm_component_utils(ddr3_base_test)
    
    //Environment instance
    ddr3_env env;
    
    // Constructor  
    function new(string name = "ddr3_base_test" ,uvm_component parent);
      super.new(name,parent);
    endfunction :new
    
    // Build Phase 
    virtual function void build_phase(uvm_phase phase);
      super.build_phase(phase);
     
      env = ddr3_env::type_id::create("env", this);
    endfunction
    
  endclass : ddr3_base_test

`endif
--------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_read_test.sv
// Author:      Mohamed Ehab
// Date:        2025-03-25
// Description: Read test class. Tests the read command respond of 
//              the DDR3 Controller.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_READ_TEST_SV
  `define DDR3_READ_TESTE_SV

class ddr3_read_test extends ddr3_base_test ;
  `uvm_component_utils(ddr3_read_test)
  
  // Sequence Declaration 
  ddr3_cpu_read_seq read_seq ;
  
  // Constructor
  function new(string name = "ddr3_read_test",uvm_component parent=null);
    super.new(name,parent);
  endfunction

  // Build Phase 
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    read_seq = ddr3_cpu_read_seq::type_id::create("read_seq");
  endfunction

  // Run Phase  
  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    
    repeat (1) read_seq.start(env.cpu_agent.sequencer);
      
    phase.drop_objection(this);
  endtask

endclass : ddr3_read_test

`endif
-----------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_test_pkg.sv
// Author:      Mohamed Ehab
// Date:        2025-03-24
// Description: Test package.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_TEST_PKG_SV
  `define DDR3_TEST_PKG_SV

  `include "uvm_macros.svh"
  `include "ddr3_env_pkg.sv"

  package ddr3_test_pkg;
    import uvm_pkg::*;
    import ddr3_env_pkg::*;
    import ddr3_cpu_pkg::*;

    `include "ddr3_base_test.sv"
    `include "ddr3_write_test.sv"
    `include "ddr3_read_test.sv"

  endpackage

`endif
----------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// File:        ddr3_write_test.sv
// Author:      Mohamed Ehab
// Date:        2025-03-25
// Description: Write test class. Tests the write command respond of 
//              the DDR3 Controller.
///////////////////////////////////////////////////////////////////////////////

`ifndef DDR3_WRITE_TEST_SV
  `define DDR3_WRITE_TESTE_SV

class ddr3_write_test extends ddr3_base_test ;
  `uvm_component_utils(ddr3_write_test)
  
  // Sequence Declaration 
  ddr3_cpu_write_seq write_seq ;
  
  // Constructor
  function new(string name = "ddr3_write_test",uvm_component parent=null);
    super.new(name,parent);
  endfunction

  // Build Phase 
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    write_seq = ddr3_cpu_write_seq::type_id::create("write_seq");
  endfunction

  // Run Phase  
  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    
    repeat (1) write_seq.start(env.cpu_agent.sequencer);
      
    phase.drop_objection(this);
  endtask

endclass : ddr3_write_test

`endif
-----------------------------------------------------------------------------------
