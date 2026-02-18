interface axi_if(input aclk);

	
	logic [3:0]awid, awlen, wid, wstrb, bid, arid, arlen, rid;
//	logic reset;
	logic [31:0]awaddr, wdata, araddr, rdata;
	logic [2:0]awsize, arsize;
	logic [1:0]awburst, arburst, bresp, rresp;
	logic awvalid, awready, arready, arvalid, bvalid, bready, rlast, rvalid, rready, wlast, wvalid, wready;


clocking mtr_drv@(posedge aclk);
	default input #1 output #0;

//	output reset;
	input wready;
	input awready;
	input bid;
	input bresp;
	input bvalid;
	output awid;
	output awaddr;
	output awlen;
	output awsize;
	output awburst;
	output awvalid;
	output wid;
	output wdata;
	output wstrb;
	output wlast;
	output wvalid;
	output bready;
	output araddr;
	output arid;
	output arlen;
	output arsize;
	output arburst;
	output arvalid;
	output rready;
	input rid;
	input rdata;	
	input rresp;
	input rlast;
	input rvalid; 
	
endclocking

clocking mtr_mon@(posedge aclk);
	default input #1 output #0;
	
//	input reset;
	input wready;
	input awready;
	input bid;
	input bresp;
	input bvalid;
	input  awid;
	input  awaddr;
	input  awlen;
	input  awsize;
	input  awburst;
	input  awvalid;
	input  wid;
	input  wdata;
	input  wstrb;
	input  wlast;
	input  wvalid;
	input  bready;
	input  araddr;
	input  arid;
	input  arlen;
	input  arsize;
	input  arburst;
	input  arvalid;
	input  rready;
	input rid;
	input rdata;	
	input rresp;
	input rlast;
	input rvalid; 
	
endclocking

clocking slv_drv@(posedge aclk);
	
	default input #1 output #0;
	
//	output reset;
	input awid;
	input awaddr;
	input awlen;
	input awsize;
	input awburst;
	input awvalid;
	input wid;
	input wdata;
	input wstrb;
	input wlast;
	input wvalid;
	input bready;
	output bid;
	input araddr;
	input arlen;
	input arsize;
	input arburst;
	input arvalid;
	input rready;
	output rvalid;
	output rlast;
	output rresp;
	output rdata;
	output rid;
	output arready;
	output bvalid;
	output bresp;
	output wready;
	output awready;

endclocking 

clocking slv_mon@(posedge aclk);
	
	default input #1 output #0;
	
//	input reset;
	input awid;
	input awaddr;
	input awlen;
	input awsize;
	input awburst;
	input awvalid;
	input wid;
	input wdata;
	input wstrb;
	input wlast;
	input wvalid;
	input bready;
	input  bid;
	input araddr;
	input arlen;
	input arsize;
	input arburst;
	input arvalid;
	input rready;
	input  rvalid;
	input  rlast;
	input  rresp;
	input  rdata;
	input  rid;
	input  arready;
	input  bvalid;
	input  bresp;
	input  wready;
	input  awready;

endclocking 

modport MTR_DRV(clocking mtr_drv);
modport MTR_MON(clocking mtr_mon);
modport SLV_DRV(clocking slv_drv);
modport SLV_MON(clocking slv_mon);
	
endinterface
===================================================================
  tb/top.sv
=========================================================
module top;

	bit clock = 1'b1;
	//bit reset = 1'b0;
	import uvm_pkg::*;
	import axi_test_pkg::*;

	axi_if INTERFACE(clock);
	
	

	initial 
		begin
			uvm_config_db #(virtual axi_if)::set(null, "*", "vif", INTERFACE);
			run_test();
		end

	always
		#10	clock = ~clock;

endmodule 
==============================================
  tb/virtual_sequencer.sv
  ==========================
  class axi_virtual_sequencer extends uvm_sequencer #(uvm_sequence_item);
		
	`uvm_component_utils(axi_virtual_sequencer)
	
	master_sequencer mtr_sqr[];
	slave_sequencer slv_sqr[];
	axi_env_config env_cfg;
	
extern function new(string name = "axi_virtual_sequencer", uvm_component parent);
extern function void build_phase(uvm_phase phase);

endclass

	
	function axi_virtual_sequencer::new(string name = "axi_virtual_sequencer", uvm_component parent);
		super.new(name, parent);
	endfunction 
	
	function void axi_virtual_sequencer::build_phase(uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(axi_env_config)::get(this, "", "axi_env_config", env_cfg))
			`uvm_fatal("VIRTUL_SEq", "cannot get inside the virtual_sequencer")
		
		mtr_sqr = new[env_cfg.no_of_mtr_agts];
		slv_sqr = new[env_cfg.no_of_slv_agts];
	
	endfunction	
  ========================================================
  tb/slave_agt_cfg.sv
  =======================================================
  class slave_agt_cfg extends uvm_object;

	`uvm_object_utils(slave_agt_cfg)

	uvm_active_passive_enum is_active = UVM_ACTIVE;
	int no_of_slv_agts = 1;

	virtual axi_if vif;


	
extern function new(string name = "slave_agt_cfg");
		
endclass

	function slave_agt_cfg::new(string name = "slave_agt_cfg");
		super.new(name);
	endfunction
==============================================
	tb/master_agt_cfg.sv
  ===============================================
	class master_agt_cfg extends uvm_object;
	
	`uvm_object_utils(master_agt_cfg)

	uvm_active_passive_enum is_active = UVM_ACTIVE;

		int no_of_mtr_agts = 1;

	virtual axi_if vif;




extern function new(string name = "master_agt_cfg");
		
endclass

	function master_agt_cfg::new(string name = "master_agt_cfg");
		super.new(name);
	endfunction
  ======================================================================
tb/axi_virtual_seq.sv
===========================================================================
class axi_virtual_seq extends uvm_sequence #(uvm_sequence_item);
	
	`uvm_object_utils(axi_virtual_seq)
	
	axi_virtual_sequencer v_seqr;
	axi_env_config env_cfg;
	master_sequencer mtr_sqr[];
	slave_sequencer slv_sqr[];
	
extern function new(string name = "axi_virtual_seq");
extern task body();
endclass

	function axi_virtual_seq::new(string name = "axi_virtual_seq");
		super.new(name);
	endfunction 


	task axi_virtual_seq::body();
		if(!uvm_config_db #(axi_env_config)::get(null,get_full_name(),"axi_env_config", env_cfg))
			`uvm_fatal("sequence_v","cannot get inside virtual sequence")
		if(!$cast(v_seqr, m_sequencer))
			begin
				`uvm_error("CASTING", "cannot perform casting of sequencer and m_sequencer");
			end
		mtr_sqr = new[env_cfg.no_of_mtr_agts];
		slv_sqr = new[env_cfg.no_of_slv_agts];
	
	foreach(mtr_sqr[i])
		mtr_sqr[i] = v_seqr.mtr_sqr[i];
	foreach(slv_sqr[i])
		slv_sqr[i] = v_seqr.slv_sqr[i];
	endtask

	
class first_virtual_seq extends axi_virtual_seq;
	

	`uvm_object_utils(first_virtual_seq)

	first_master_seq mtr_seqI;
	first_slave_seq slv_seqI;
	
	extern function new(string name = "first_virtual_seq");
	extern task body();
	
endclass
	
	function first_virtual_seq::new(string name = "first_virtual_seq");
		super.new(name);
	endfunction 

	task first_virtual_seq::body();
		super.body();
			mtr_seqI = first_master_seq::type_id::create("mtr_seqI");
		slv_seqI = first_slave_seq::type_id::create("slv_seqI");
		fork 

		
			mtr_seqI.start(mtr_sqr[0]);
		
			slv_seqI.start(slv_sqr[0]);
		join 

	endtask
==============================================================================================
    tb/axi_scoreboard.sv
================================================================================
class axi_scoreboard extends uvm_scoreboard;

	`uvm_component_utils(axi_scoreboard)

extern function new(string name = "axi_scoreboard", uvm_component parent);
endclass

	function axi_scoreboard::new(string name = "axi_scoreboard", uvm_component parent);
		super.new(name ,parent);
	endfunction 
======================================================================
  tb/axi_env_config.sv
=================================================================
  class axi_env_config extends uvm_object;

	`uvm_object_utils(axi_env_config)


	master_agt_cfg m_cfg[];
	slave_agt_cfg slv_cfg[];

//	virtual axi_if vif;
//	uvm_active_passive_enum is_active = UVM_ACTIVE;
//	bit has_functional_coverage = 1;
	int has_sb = 1;
	int has_v_seqr = 1;
	int no_of_mtr_agts = 1;
	int no_of_slv_agts = 1;
	int no_of_mtops =1; 
	int no_of_stops = 1;

extern function new(string name = "axi_env_config");
endclass

	function axi_env_config::new(string name = "axi_env_config");
		super.new(name);
	endfunction 
=================================================================
  tb/avi_environment.sv
  ========================================================
class axi_environment extends uvm_env;
	
	`uvm_component_utils(axi_environment)
	
	axi_env_config env_cfg;
//	slave_agt_cfg slv_cfg[];
//	master_agt_cfg m_cfg[];

	master_agt_top mtop[];
	slave_agt_top slvtop[];
		
	axi_scoreboard sb;
	axi_virtual_sequencer v_seqr;


	

extern function new(string name = "axi_environment", uvm_component parent);
extern function void build_phase (uvm_phase phase);
extern function void connect_phase(uvm_phase phase);
endclass
	function axi_environment::new(string name = "axi_environment", uvm_component parent);
		super.new(name, parent);
	endfunction 

	function void axi_environment::build_phase(uvm_phase phase);
		
		super.build_phase(phase);
		if(!uvm_config_db #(axi_env_config)::get(this, "", "axi_env_config", env_cfg))
			`uvm_fatal("CONFIG DB", "cannot get inside the environmeint from test")
		
		if(env_cfg.has_sb)
			sb = axi_scoreboard::type_id::create("sb", this);
	
		if(env_cfg.has_v_seqr)
			v_seqr = axi_virtual_sequencer::type_id::create("v_seqr", this);
		
		mtop = new[env_cfg.no_of_mtops];
		foreach(mtop[i])
			begin
				mtop[i] = master_agt_top::type_id::create($sformatf("mtop[%0d]",i), this);
				uvm_config_db #(master_agt_cfg)::set(this, $sformatf("mtop[%0d]*",i), "master_agt_cfg", env_cfg.m_cfg[i]);
			end

		slvtop = new[env_cfg.no_of_stops];
		foreach(slvtop[i])
			begin
				slvtop[i] = slave_agt_top::type_id::create($sformatf("slvtop[%0d]",i), this);
				uvm_config_db #(slave_agt_cfg)::set(this, $sformatf("slvtop[%0d]*", i), "slave_agt_cfg", env_cfg.slv_cfg[i]);
			end

		

	endfunction 

	function void axi_environment::connect_phase(uvm_phase phase);
		//for(int i =0; i<env_cfg.no_of_mtops; i= i+1)
		//	begin
		//		for(int j = 0; j< env_cfg.no_of_mtr_agts; j = j+1)
		//			begin
						v_seqr.mtr_sqr[0] = mtop[0].m_agt[0].mtr_sqr;
		//			end
		//	end
//		for(int m =0; m<env_cfg.no_of_stops; m= m+1)
//			begin
//				for(int n = 0; n< env_cfg.no_of_slv_agts; n = n+1)
//					begin

						v_seqr.slv_sqr[0] = slvtop[0].slv_agt[0].slv_sqr;
//					end
//			end
	endfunction 
======================================================================================================================
	master_agt_top/master_agent.sv
================================================================================================================
  class master_agent extends uvm_agent;
	
	`uvm_component_utils(master_agent)

	master_agt_cfg m_cfg;
	master_driver m_drv;
	master_sequencer mtr_sqr;
	master_monitor m_mon;
	
	
	extern function new(string name = "master_agent", uvm_component parent);
	extern function void build_phase(uvm_phase phase);
	extern function void connect_phase(uvm_phase phase);
	
	endclass

	function master_agent::new(string name = "master_agent", uvm_component parent);

		super.new(name, parent);
	endfunction

	function void master_agent::build_phase(uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(master_agt_cfg)::get(this, "", "master_agt_cfg", m_cfg))
			`uvm_fatal("config db", "cannopt get inside agents ")
		if(m_cfg.is_active == UVM_ACTIVE)
			begin
			m_drv = master_driver::type_id::create("m_drv", this);
			mtr_sqr = master_sequencer::type_id::create("mtr_sqr", this);
			end	
		m_mon = master_monitor::type_id::create("m_mon", this);
	endfunction 

	function void master_agent::connect_phase(uvm_phase phase);
		if(m_cfg.is_active == UVM_ACTIVE)
			begin
				m_drv.seq_item_port.connect(mtr_sqr.seq_item_export);
			end
	endfunction 
    =============================================================================
master_agt_top/master_agt_top.sv
    ========================================================================
class master_agt_top extends uvm_env;
	
	`uvm_component_utils(master_agt_top)

	master_agt_cfg m_cfg;
	
	master_agent m_agt[];

	
	extern function new(string name = "master_agt_top", uvm_component parent);
	extern function void build_phase(uvm_phase phase);

endclass 

	function master_agt_top::new(string name = "master_agt_top", uvm_component parent);
		super.new(name, parent);
	endfunction 
	
	function void master_agt_top::build_phase (uvm_phase phase);
		super.build_phase(phase);
	//	m_cfg = master_agt_cfg::type_id::create("m_cfg");
		if(!uvm_config_db #(master_agt_cfg)::get(this, "", "master_agt_cfg", m_cfg))
			`uvm_fatal("CONFIG DB", "cannot get inside agent tops")
//	`uvm_info("DISPLAY",$sformatf("value of no_of_mtr_agts", m_cfg.no_of_mtr_agts), UVM_NONE)
	m_agt = new[m_cfg.no_of_mtr_agts];
	foreach(m_agt[i])
		begin
		m_agt[i] = master_agent::type_id::create($sformatf("m_agt[%0d]", i), this);
		uvm_config_db #(master_agt_cfg)::set(this, $sformatf("m_agt[%0d]*.", i), "master_agt_cfg", m_cfg);
		end
		
	endfunction
========================================================================================================
master_agt_top/master_driver.sv
==========================================================================================================
class master_driver extends uvm_driver #(master_xtn);
	
	`uvm_component_utils(master_driver)

	virtual axi_if.MTR_DRV vif;

		master_agt_cfg m_cfg;
		semaphore sem_addr = new(1);
		semaphore sem_data = new(1);	
		semaphore sem_resp = new(1);

		longint q[$];
		longint burst_vl;

extern function new(string name = "master_driver", uvm_component parent);
extern function void connect_phase (uvm_phase phase);
extern function void build_phase (uvm_phase phase);
extern task run_phase(uvm_phase phase);
extern task send_to_dut(master_xtn xtnI);
extern task address_channel(master_xtn xtnI);
extern task write_data_chnl(master_xtn xtnI);
extern task resp_channel(master_xtn xtnI);
		
endclass 
		
	function master_driver::new(string name = "master_driver", uvm_component parent);
		super.new(name, parent);
	endfunction 

	function void master_driver::build_phase(uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(master_agt_cfg)::get(this, "", "master_agt_cfg", m_cfg))
			`uvm_fatal ("MASTER DRIVER", "cannot get inside driver of master")
	endfunction 
	
	function void master_driver::connect_phase(uvm_phase phase);
		vif=m_cfg.vif;
	endfunction 

	task master_driver::run_phase(uvm_phase phase);
		forever
			begin
				seq_item_port.get_next_item(req);
				send_to_dut(req);
				seq_item_port.item_done();
			end
	endtask 
		
	task master_driver::send_to_dut(master_xtn xtnI);
				xtnI.print();
		fork 
			begin
			sem_addr.get(1);
				address_channel(xtnI);
					
			sem_addr.put(1);
			sem_resp.put(1);
			end

			
			begin
			//@(vif.mtr_drv);
		 	sem_data.get(1);
						
				write_data_chnl(xtnI);	
				
			sem_data.put(1);
			sem_resp.put(1);
			end

				
			begin
			sem_resp.get(3);
				resp_channel(xtnI);
			sem_resp.put(1);
			end

		join_any

	endtask 


	task master_driver::address_channel(master_xtn xtnI);
			
				q.push_front(xtnI.burst_ln);
			//repeat(2)	@(vif.mtr_drv);
				
				$display("passed the key to first_sem inside master driver ===============at time=%d",$time);
				vif.mtr_drv.awvalid <= 1'b1;
				wait(vif.mtr_drv.awready)
				vif.mtr_drv.awid <= xtnI.awid;
				vif.mtr_drv.awaddr <= xtnI.awaddr;

				vif.mtr_drv.awlen <= xtnI.awlen;
				$display("MASTER%%$####time at which the value in the queue is pushed =%d ADDRESS CHANNEL" ,$time);

				$display("MASTER%%%####the value of the queue=====%p and the burst_ln=====%d ADDRESS CHANNEL", q, xtnI.burst_ln);

				vif.mtr_drv.awsize <= xtnI.awsize;

				vif.mtr_drv.awburst <= xtnI.awburst;
				@(vif.mtr_drv);
				
				vif.mtr_drv.awvalid <= 1'b0;
				//@(vif.mtr_drv);

				

	endtask 

	task master_driver::write_data_chnl(master_xtn xtnI);
			
				$display("passed the key to second sem inside master driver=================at time=%d",$time);
				$display("valure of the data inside the queue Q_DATA=%p", xtnI.q_data);	
					repeat(4)
					@(vif.mtr_drv);
				
						vif.mtr_drv.wvalid <= 1'b1;
						wait(vif.mtr_drv.wready)
						burst_vl = q.pop_back();
	
		
						$display("MASTER%%%####time at which the value in the queue is popped=%d and after popping the value of queue=%pDATA CHANNEL",$time, q);

				$display("value of the burst length variable in the queue -------$$$$$$$$$=%d",burst_vl );
				for(int i=1; i<=(burst_vl); i++)
					begin
						if(i == burst_vl)
							begin
								vif.mtr_drv.wlast <= 1'b1;
							end
						//vif.mtr_drv.wid <= xtnI.wid;
					
						vif.mtr_drv.wdata <= xtnI.q_data.pop_back();
						vif.mtr_drv.wstrb <= xtnI.q_strb.pop_back();
						@(vif.mtr_drv);
						

					end
						vif.mtr_drv.wlast <= 1'b0;
						vif.mtr_drv.wvalid <= 1'b0;
					//	vif.mtr_drv.wid <= 0;
						vif.mtr_drv.wdata <=0;
						vif.mtr_drv.wstrb <= 0;
					/*	repeat(2)
						@(vif.mtr_drv);*/
					

	endtask 

	task master_driver::resp_channel(master_xtn xtnI);
	/*	repeat(2)
		@(vif.mtr_drv);*/
		wait(!vif.mtr_drv.awready && !vif.mtr_drv.wready)
		repeat(2)
		@(vif.mtr_drv);
		vif.mtr_drv.bready <= 1'b1;
		wait(vif.mtr_drv.bvalid)
		xtnI.bid = vif.mtr_drv.bid;
		xtnI.bresp = vif.mtr_drv.bresp;
		//repeat(2)
		
		@(vif.mtr_drv);
		wait(vif.mtr_drv.wready)
		
		vif.mtr_drv.bready <= 1'b0;
	
	endtask
=================================================================
master_agt_top/master_monitor.sv
=================================================================
class master_monitor extends uvm_monitor;
		
	`uvm_component_utils(master_monitor)
virtual axi_if vif;
master_agt_cfg m_cfg;

extern function new(string name = "master_monitor", uvm_component parent);
extern function void build_phase(uvm_phase phase);
extern function void connect_phase(uvm_phase phase);

endclass

	function master_monitor::new(string name = "master_monitor", uvm_component parent);

		super.new(name, parent);
	endfunction 

	function void master_monitor::build_phase(uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(master_agt_cfg)::get(this, "", "master_agt_cfg", m_cfg))
			`uvm_fatal("MASTER MONITOR", "cannot get inside montior of master")
	endfunction 

	function void master_monitor::connect_phase(uvm_phase phase);
		vif = m_cfg.vif;
	endfunction 
=========================================================================
master_agt_top/master_sequence.sv
========================================================================
class master_sequence extends uvm_sequence #(master_xtn);
	
	`uvm_object_utils(master_sequence)

extern function new(string name = "master_sequence");
endclass

	function master_sequence::new(string name = "master_sequence");
	
		super.new(name);
	 endfunction 
	
	
class first_master_seq extends master_sequence;
		
	`uvm_object_utils(first_master_seq)
	
extern function new(string name = "first_master_seq");
extern task body();

endclass

	function first_master_seq::new(string name = "first_master_seq");
	
		super.new(name);
	endfunction 

	task first_master_seq::body();
		
	repeat(3)
		begin

			req = master_xtn::type_id::create("req");
			start_item(req);


			assert(req.randomize() with { awburst==2'd1;});
			finish_item(req);
		end
	endtask 
===============================================================================
master_agt_top/master_sequencer.sv
==============================================================================
class master_sequencer extends uvm_sequencer #(master_xtn);
	
	`uvm_component_utils(master_sequencer)
	
extern function new(string name = "master_sequencer", uvm_component parent);
endclass

	function master_sequencer::new(string name = "master_sequencer", uvm_component parent);
		super.new(name, parent);
	endfunction 
=================================================================================
master_agt_top/master_xtn.sv
=================================================================================
class master_xtn extends uvm_sequence_item;
	
	`uvm_object_utils(master_xtn)

	//rand logic [3:0]awid, bid, rid, wid;
	rand logic [3:0]awid, awlen, arid, arlen;
	rand logic [3:0]wid;
	logic [3:0]rid, bid;
	rand logic [31:0]awaddr, araddr;
	rand logic [31:0]wdata[];
	logic [31:0]rdata[];
	rand logic [2:0]awsize, arsize;
	rand logic [2:0]n;
	rand logic [1:0]awburst, arburst;
	logic [3:0]wstrb;
	logic [1:0] bresp, rresp;
	logic awvalid, awready, arready, arvalid, bvalid, bready, rlast, rvalid, rready, wlast, wvalid, wready;

	 	logic [31:0]al_addr, next_addr;
		logic [7:0]num_bytes;
		logic [7:0]burst_ln;
		logic [1:0]burst_type;
		logic [2:0]size_addr;
		logic [31:0]wrap_b;
		logic [3:0]strb;
		logic [31:0]extra_addr;
		
		int q_strb[$];
		int q_data[$];

//	constraint EVEN_N{n inside {[1:4]};}	
//	constraint BURST{if(awburst == 2'b10) (awlen == (2**n - 1));} // randomization for length of burst for wrapping 
	constraint DATA{wdata.size == (awlen +1'b1);}
	//constraint RDATA{rdata.size == (awlen +1'b1);}
/*	constraint WID{wid.size == ((awlen +1'b1)*(2**awsize));}
	constraint BID{bid.size == ((awlen +1'b1)*(2**awsize));}
	constraint RID{rid.size == ((awlen +1'b1)*(2**awsize));}*/
	constraint ARRAY{foreach(wdata[i])
				wdata[i]<30;}
	constraint SIZE{awsize inside {[0:2]};}


extern function new(string name = "master_xtn");
extern function void do_print(uvm_printer printer); 
extern function void strb_clc();
extern function void post_randomize();

endclass

	function master_xtn::new(string name = "master_xtn");
		super.new(name);
	endfunction 


	function void master_xtn::do_print(uvm_printer printer);
		printer.print_field("address ID", this.awid, 4, UVM_DEC);
		printer.print_field("burst length ", this.awlen, 4, UVM_DEC);
		printer.print_field("write ID", this.wid, 4, UVM_DEC);
		
				printer.print_field("wstrb", this.wstrb, 4, UVM_DEC);
		
		printer.print_field("response ID", this.bid, 4, UVM_DEC);
		printer.print_field("address read ID", this.arid, 4, UVM_DEC);
		printer.print_field("transfer length read", this.arlen, 4, UVM_DEC);
		printer.print_field("read ID", this.rid, 4, UVM_DEC);
		printer.print_field("write address", this.awaddr, 32, UVM_HEX);
		foreach(wdata[i])
			begin
				printer.print_field($sformatf("wdata[%0d]",i), this.wdata[i], 32, UVM_DEC);
			end
		printer.print_field("read address", this.araddr, 32, UVM_DEC);
		foreach(rdata[i])
			begin
				printer.print_field($sformatf("rdata[%0d]",i), this.rdata[i], 32, UVM_DEC);
			end
		printer.print_field("write data transfer size", this.awsize, 3, UVM_DEC);
		printer.print_field("read data transfer size", this.arsize, 3, UVM_DEC);
	
		printer.print_field("write data burst", this.awburst, 2, UVM_DEC);
		printer.print_field("read data burst", this.arburst, 2, UVM_DEC);
		
	/*	printer.print_field("write response", this.bresp, 2, UVM_DEC);
		printer.print_field("read response", this.rresp, 2, UVM_DEC);
	
		printer.print_field("write address valid", this.awvalid, 1, UVM_DEC);
		printer.print_field("write valid", this.wvalid, 1, UVM_DEC);
		printer.print_field("read address valid", this.arvalid, 1, UVM_DEC);
		printer.print_field("read valid", this.rvalid, 1, UVM_DEC);
		printer.print_field("write response valid", this.bvalid, 1, UVM_DEC);
		printer.print_field("write address ready", this.awready, 1, UVM_DEC);
		printer.print_field("read address ready", this.arready, 1, UVM_DEC);
		printer.print_field("write data ready", this.wready, 1, UVM_DEC);
		printer.print_field("read data ready", this.rready, 1, UVM_DEC);
		printer.print_field("response ready", bready, 1, UVM_DEC);
		printer.print_field("write data last", wlast, 1, UVM_DEC);
		printer.print_field("read data last", rlast, 1, UVM_DEC);*/

	endfunction 

	function void master_xtn::post_randomize();	

		burst_ln = awlen + 1'b1;
		burst_type = awburst;
		size_addr = awsize;
		num_bytes = 2**(size_addr);

		al_addr = awaddr;
		
		wrap_b = (awaddr/(num_bytes*burst_ln))*(num_bytes*burst_ln);

		


		case(burst_type)
		2'b00: begin
				for(int i = 1; i<=burst_ln; i=i+1)
					begin
					next_addr = al_addr;
					strb_clc();
					q_strb.push_front(wstrb);
					
					$display("the strobe wstrb =%h for size =%d, burst_type =%d, and length =%d for addr =%h", wstrb, size_addr, burst_type, burst_ln, next_addr );				
					q_data.push_front(wdata[i-1]);
					


					end
			end
		2'b01: begin
	
				for(int i = 1; i<=(burst_ln); i = i+1)
					begin
						next_addr = al_addr + ((i-1)*num_bytes);
					strb_clc();

					q_strb.push_front(wstrb);



					$display("the strobe wstrb =%h for size =%d, burst_type =%d, and length =%d for addr =%h", wstrb, size_addr, burst_type, burst_ln, next_addr );
					q_data.push_front(wdata[i-1]);
					




					end
		      end

		2'b10: begin		
				extra_addr = al_addr + num_bytes*(1 - burst_ln);
				for(int i = 1; i <= burst_ln; i++)
					begin
								if(extra_addr == (wrap_b + (num_bytes*burst_ln)))
								begin
									next_addr = wrap_b;
									strb_clc();
									$display("the strobe for wrap boundary  wwstrb =%b for size =%d, burst_type =%d, and length =%d for addr =%h", wstrb, size_addr, burst_type, burst_ln, next_addr);
								end
								else
								begin	
									next_addr = al_addr + ((i-1)*num_bytes) - (num_bytes*burst_ln);
									strb_clc();
									$display("the strobe wstrb =%b for size =%d, burst_type =%d, and length =%d for addr =%h", wstrb, size_addr, burst_type, burst_ln, next_addr);
								end
					end
			end
	endcase
						 
	endfunction 	
		  	
	function void master_xtn::strb_clc();
		 case(size_addr)
					
						3'b000: case(next_addr[1:0])
							
							2'b00: wstrb = 4'b0001;
							2'b01: wstrb = 4'b0010;
							2'b10: wstrb = 4'b0100;
							2'b11: wstrb = 4'b1000;
							endcase
						3'b001: case(next_addr[1:0])
							
							2'b00: wstrb = 4'b0011;
							2'b10: wstrb = 4'b0010;
		
							2'b01: wstrb = 4'b1100;
							2'b11: wstrb = 4'b1000;
							endcase
						default: case(next_addr[1:0])
							2'b00: wstrb = 4'b1111;
							2'b01: wstrb = 4'b1110;
							2'b10: wstrb = 4'b1100;
							2'b11: wstrb = 4'b1000;
				     			endcase
 		endcase
		
	endfunction 
======================================================================
slave_agt_top/slave_agent.sv
=====================================================================
class slave_agent extends uvm_agent;
	
	`uvm_component_utils(slave_agent)

	slave_agt_cfg slv_cfg;

	slave_driver slv_drv;
	slave_sequencer slv_sqr;
	slave_monitor slv_mon;
	
	
	extern function new(string name = "slave_agent", uvm_component parent);
	extern function void build_phase(uvm_phase phase);
	extern function void connect_phase(uvm_phase phase);

	
	endclass

	function slave_agent::new(string name = "slave_agent", uvm_component parent);

		super.new(name, parent);
	endfunction

	function void slave_agent::build_phase(uvm_phase phase);
		super.build_phase(phase);

		if(!uvm_config_db #(slave_agt_cfg)::get(this, "", "slave_agt_cfg", slv_cfg))
			`uvm_fatal("config db", "cannopt get inside agents ")
		if(slv_cfg.is_active == UVM_ACTIVE)
			begin
			slv_drv = slave_driver::type_id::create("slv_drv", this);
			slv_sqr = slave_sequencer::type_id::create("slv_sqr", this);
			end
		slv_mon = slave_monitor::type_id::create("slv_mon", this);
	endfunction 

	function void slave_agent::connect_phase(uvm_phase phase);
		if(slv_cfg.is_active == UVM_ACTIVE)
			begin
					slv_drv.seq_item_port.connect(slv_sqr.seq_item_export);
			end
	endfunction 
===================================================================================
slave_agt_top/slave_agt_top.sv
==================================================================================
class slave_agt_top extends uvm_env;
	
	`uvm_component_utils(slave_agt_top)

	slave_agt_cfg slv_cfg;
	slave_agent slv_agt[];

	
	extern function new(string name = "slave_agt_top", uvm_component parent);
	extern function void build_phase(uvm_phase phase);

endclass 

	function slave_agt_top::new(string name = "slave_agt_top", uvm_component parent);
		super.new(name, parent);
	endfunction 
	
	function void slave_agt_top::build_phase (uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(slave_agt_cfg)::get(this, "", "slave_agt_cfg", slv_cfg))
			`uvm_fatal("CONFIG DB", "cannot get inside agent tops")
	
	slv_agt = new[slv_cfg.no_of_slv_agts];
	foreach(slv_agt[i])
		begin
		slv_agt[i] = slave_agent::type_id::create($sformatf("slv_agt[%0d]", i), this);
		uvm_config_db #(slave_agt_cfg)::set(this, $sformatf("slv_agt[%0d]*.", i), "slave_agt_cfg", slv_cfg);
		end

	endfunction
====================================================================================================
slave_agt_top/slave_driver.sv
==================================================================================================
class slave_driver extends uvm_driver #(slave_xtn);
		
	`uvm_component_utils(slave_driver)

	virtual axi_if.SLV_DRV vif;

	slave_agt_cfg slv_cfg;

	slave_xtn clcxtn;

	semaphore sem_addr = new(1);	
	semaphore sem_data = new(1);
	semaphore sem_resp = new(1);
			int q1[$];
			int q2[$];
			longint q3[$];
			int q_awid[$];

		logic [31:0]next_addr, al_addr;
		logic [7:0]num_bytes; 
		longint slv_burst_ln; 
		logic [1:0]burst_type;
		logic [2:0]size_addr;
		logic [31:0] wrap_b;
			logic [31:0]addr_array[];
		logic [31:0]extra_addr;
		longint burst_ln;
		int a_wid;

	
extern function new(string name = "slave_driver", uvm_component parent );
extern function void connect_phase (uvm_phase phase);
extern function void build_phase (uvm_phase phase);
extern task run_phase(uvm_phase phase);
extern task send_to_dut(slave_xtn xtnI);
extern task write_data_chnl(slave_xtn xtnII);
extern task address_channel();
extern task resp_channel();


endclass

	function slave_driver::new(string name = "slave_driver", uvm_component parent);

		super.new(name, parent);
	endfunction 

	function void slave_driver::build_phase(uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(slave_agt_cfg)::get(this, "", "slave_agt_cfg", slv_cfg))
			`uvm_fatal ("SLAVE DRIVER", "cannot get inside driver of slave")
	endfunction 
	
	function void slave_driver::connect_phase(uvm_phase phase);
		vif=slv_cfg.vif;
	endfunction 

	task slave_driver::run_phase(uvm_phase phase);
		forever
			begin
				seq_item_port.get_next_item(req);
				send_to_dut(req);
				seq_item_port.item_done();
			end
	endtask 
	

	task slave_driver::send_to_dut(slave_xtn xtnI);//, slave_xtn clcxtn);

		fork 

			begin
			sem_addr.get(1);
				clcxtn = slave_xtn::type_id::create("clcxtn");
				address_channel();
			sem_addr.put(1);
			sem_resp.put(1);
			end
			
			

				begin
			
				sem_data.get(1);	
						
					write_data_chnl(clcxtn);			
				sem_data.put(1);
				sem_resp.put(1);
							
				end
			
			begin
			sem_resp.get(3);
			
				resp_channel();
			sem_resp.put(1);
			end
				
		join_any 
					
			
	endtask





	task slave_driver::write_data_chnl(slave_xtn xtnII);
					$display("passed the key to second_sem inside slave driver ========at time=%d",$time);
						repeat(2)
						@(vif.slv_drv);

							vif.slv_drv.wready <= 1'b1;
							//@(vif.slv_drv);
							wait(vif.slv_drv.wvalid)
						//repeat(1)
						//@(vif.slv_drv);
						$display("%%%%%%%%%%%%%%%%%%%%%%%%%%% data channel slave time=%d",$time);
						burst_ln = q3.pop_back();
						$display("SLAVE$$$$time at which the value in the queue is popped=%d and after popping the value of queue3333=%p DATA CHANNEL",$time, q3);
					 xtnII.wdata = new[burst_ln];
					$display("SLAVE########value of burst_ln=%d DATA CHANNEL",burst_ln);
					for(int i=0 ; i<(burst_ln); i++)
						begin
							$display("------------------slave driver after asserting WREAdy and waiting for WVALID--------------");
							$display("%%%%%%%%%%%%%%%%%%%%%%%%%%% data channel slave time=%d",$time);
							if(i==(burst_ln-1))
							vif.slv_drv.wready <= 0;
	
							//xtnII.wid = vif.slv_drv.wid;
							xtnII.wdata[i] = vif.slv_drv.wdata;
							xtnII.wstrb= vif.slv_drv.wstrb;
							xtnII.q_strb.push_front(xtnII.wstrb);
							$display("value of the strobe inside queue=%p",xtnII.q_strb);
							@(vif.slv_drv);
						end
						//@(vif.slv_drv);
							$display("memory ===============================%p at time=%d",	xtnII.wdata, $time);
					//	vif.slv_drv.wready <= 0;
					//	repeat(2)
						//@(vif.slv_drv);
					//xtnII.print();

	endtask 
		
	task slave_driver::address_channel();
		parameter FIXED = 2'b00,
			  INCR  = 2'b01,
			  WRAP  = 2'b10;
			$display("passed the key to first_sem inside slave driver ===========at time=%d",$time);
 	 	 	vif.slv_drv.awready  <= 1'b1;
			wait(vif.slv_drv.awvalid)
			//repeat(2)
				@(vif.slv_drv);
						
						clcxtn.awid = vif.slv_drv.awid;
						q_awid.push_front(clcxtn.awid);
						clcxtn.awaddr = vif.slv_drv.awaddr;
						clcxtn.awlen = vif.slv_drv.awlen;
						$display("value of awlen====%d at time=%d", clcxtn.awlen, $time);
						clcxtn.awsize = vif.slv_drv.awsize;		
						clcxtn.awburst = vif.slv_drv.awburst;

					al_addr = clcxtn.awaddr;
					size_addr = clcxtn.awsize;
					burst_type = clcxtn.awburst;
					slv_burst_ln = clcxtn.awlen + 1'b1;
				//	@(vif.slv_drv);
					q3.push_front(slv_burst_ln);
					$display("SLAVE$$$value of the queue Q33333333333333333 =%p at time======%d ADDRESS CHANNEL",q3, $time);
					$display("value of burst length is===++==========%d", slv_burst_ln);
					num_bytes = 2**(size_addr);
					
		wrap_b = (al_addr/(num_bytes*slv_burst_ln))*(num_bytes*slv_burst_ln);
					
			addr_array = new[(slv_burst_ln*num_bytes)];

				case(burst_type)
					FIXED : begin
							for(int i = 1; i<=slv_burst_ln; i++)
							begin
								next_addr = al_addr;
								q1.push_front(next_addr);
								$display("------------Next address for fixed burst=%h----------", next_addr);
							end
						end
					INCR : 
						begin
							for(int i = 1; i<=(slv_burst_ln); i = i+1)
								begin
								next_addr = al_addr + {(i-1)*(num_bytes)};
								q1.push_front(next_addr);
								$display("----Next address for incrementing burst =%h--------at burst no =%d----------", next_addr, i);
								end
						end
					WRAP : begin
						for(int i = 1; i <= slv_burst_ln; i++)
							begin
								
								if(next_addr == (wrap_b + (num_bytes*slv_burst_ln)))
								begin
									next_addr = wrap_b;
									q1.push_front(wrap_b);
									$display("inside for loop wrap boundary  for iteration number = %d", i);
									$display("------------Next address for wrapping burst = %h---at burst no = %d----------", next_addr,i);
								end
								else 
									begin

										next_addr = al_addr + ((i-1)*num_bytes) - (num_bytes*slv_burst_ln);
										$display("inside for loop for iteration number = %d", i);
										q1.push_front(next_addr);
										$display("------------Next address for wrapping burst = %h----at burst no = %d----------", next_addr,i);		
                  end
							end
						end
				endcase

				vif.slv_drv.awready <= 1'b0;	
				//@(vif.slv_drv);			
	endtask



	task slave_driver::resp_channel();
	/*	repeat(2)
		@(vif.slv_drv);*/
			wait(!vif.slv_drv.wvalid)
			wait(!vif.slv_drv.awvalid && !vif.slv_drv.wlast)	
			repeat(2)		
			@(vif.slv_drv);
			vif.slv_drv.bvalid <= 1'b1;
			a_wid = q_awid.pop_back();
			vif.slv_drv.bid <= a_wid;
			vif.slv_drv.bresp <= 2'b00;
		//	repeat(2)
			@(vif.slv_drv);
			wait(vif.slv_drv.wvalid)
			vif.slv_drv.bid <= 0;
			vif.slv_drv.bresp <= 2'bxx;

					
			vif.slv_drv.bvalid <= 1'b0;
	endtask 
==============================================================================================================
slave_agt_top/slave_monitor.sv
=============================================================================================================
class slave_monitor extends uvm_monitor;
		
	`uvm_component_utils(slave_monitor)
slave_agt_cfg slv_cfg;
virtual axi_if.SLV_MON vif;



extern function new(string name = "slave_monitor", uvm_component parent);
extern function void build_phase(uvm_phase phase);
extern function void connect_phase(uvm_phase phase);
endclass

	function slave_monitor::new(string name = "slave_monitor", uvm_component parent);

		super.new(name, parent);
	endfunction 


	function void slave_monitor::build_phase(uvm_phase phase);
		super.build_phase(phase);
		if(!uvm_config_db #(slave_agt_cfg)::get(this, "", "slave_agt_cfg", slv_cfg))
			`uvm_fatal("MASTER MONITOR", "cannot get inside montior of slave")
	endfunction 

	function void slave_monitor::connect_phase(uvm_phase phase);
		vif = slv_cfg.vif;
	endfunction 
=======================================================================================================
slave_agt_top/slave_sequence.sv
=================================================================================================
class slave_sequence extends uvm_sequence #(slave_xtn);
	
	`uvm_object_utils(slave_sequence)

extern function new(string name = "slave_sequence");
endclass

	function slave_sequence::new(string name = "slave_sequence");
	
		super.new(name);
	endfunction 
	
	
class first_slave_seq extends slave_sequence;
		
	`uvm_object_utils(first_slave_seq)
	
extern function new(string name = "first_slave_seq");
extern task body();

endclass

	function first_slave_seq::new(string name = "first_slave_seq");
	
		super.new(name);
	endfunction 

	task first_slave_seq::body();
		
	repeat(3)
		begin
			req = slave_xtn::type_id::create("req");
			start_item(req);
			assert(req.randomize());
			finish_item(req);
		end
	endtask 
=============================================================================
slave_agt_top/slave_sequencer.sv
=============================================================================
class slave_sequencer extends uvm_sequencer #(slave_xtn);
	
	`uvm_component_utils(slave_sequencer)
	
extern function new(string name = "slave_sequencer", uvm_component parent);
endclass

	function slave_sequencer::new(string name = "slave_sequencer", uvm_component parent);
		super.new(name, parent);
	endfunction 
================================================================================
slave_agt_top/slave_xtn.sv
================================================================================
class slave_xtn extends uvm_sequence_item;
	
	`uvm_object_utils(slave_xtn)

	//rand logic [3:0]awid, bid, rid, wid;
	logic [3:0]awid, awlen, arid, arlen;
	rand logic [3:0]rid, bid;
	logic [3:0]wid;
	logic [31:0]awaddr, araddr;
	logic [31:0]wdata[];
	rand logic [31:0] rdata[];
	logic [2:0]awsize, arsize;
	//logic [2:0]n;
	logic [1:0]awburst, arburst;
	logic [3:0]wstrb;
	logic [1:0] bresp, rresp;
	logic awvalid, awready, arready, arvalid, bvalid, bready, rlast, rvalid, rready, wlast, wvalid, wready;

	 	logic [31:0]al_addr, next_addr;
		logic [7:0]num_bytes;
		logic [4:0]burst_ln;
		logic [1:0]burst_type;
		logic [2:0]size_addr;
		logic [31:0]wrap_b;
		logic [3:0]strb;
		logic [31:0]extra_addr;
		
		int q_strb[$];

	//constraint EVEN_N{n inside {[1:4]};}	
	//constraint BURST{if(awburst == 2'b10) (awlen == (2**n - 1));} // randomization for length of burst for wrapping 
	//constraint DATA{wdata.size == (awlen +1'b1);}
//	constraint RDATA{rdata.size == (awlen +1'b1);}
//	constraint WDTA{wdata.size == (awlen + 1'b1);}
/*	constraint WID{wid.size == ((awlen +1'b1)*(2**awsize));}
	constraint BID{bid.size == ((awlen +1'b1)*(2**awsize));}
	constraint RID{rid.size == ((awlen +1'b1)*(2**awsize));}*/
//	constraint ARRAY{foreach(rdata[i])
//				rdata[i]<30;}
	//constraint SIZE{awsize inside {[0:2]};}


extern function new(string name = "slave_xtn");
extern function void do_print(uvm_printer printer); 
//extern function void strb_clc();
//extern function void post_randomize();

endclass

	function slave_xtn::new(string name = "slave_xtn");
		super.new(name);
	endfunction 


	function void slave_xtn::do_print(uvm_printer printer);
		printer.print_field("address ID", this.awid, 4, UVM_DEC);
		printer.print_field("burst length ", this.awlen, 4, UVM_DEC);
		printer.print_field("write ID", this.wid, 4, UVM_DEC);
				printer.print_field("wstrb", this.wstrb, 4, UVM_DEC);
		

		printer.print_field("response ID", this.bid, 4, UVM_DEC);
		printer.print_field("address read ID", this.arid, 4, UVM_DEC);
		printer.print_field("transfer length read", this.arlen, 4, UVM_DEC);
		printer.print_field("read ID", this.rid, 4, UVM_DEC);
		printer.print_field("write address", this.awaddr, 32, UVM_HEX);
		foreach(wdata[i])
			begin
				printer.print_field($sformatf("wdata[%0d]",i), this.wdata[i], 32, UVM_DEC);
			end
		printer.print_field("read address", this.araddr, 32, UVM_DEC);
		foreach(rdata[i])
			begin
				printer.print_field($sformatf("rdata[%0d]",i), this.rdata[i], 32, UVM_DEC);
			end
		printer.print_field("write data transfer size", this.awsize, 3, UVM_DEC);
		printer.print_field("read data transfer size", this.arsize, 3, UVM_DEC);
	
		printer.print_field("write data burst", this.awburst, 2, UVM_DEC);
		printer.print_field("read data burst", this.arburst, 2, UVM_DEC);

	endfunction 
