
`define RESET		3'b001
`define IDLE		3'b010
`define START_BIT	3'b011
`define DATA_BITS	3'b100
`define STOP_BIT	3'b101
`define PARITY_BIT	3'b110

module MS_UART_TX(
	input wire CLK,		
	input wire RESETN,
	input wire START,
	input wire TICK,
	input wire [3:0] DIN,
	
	output reg DONE,
	output reg BUSY,
	output reg DOUT);
	
	
	wire [4:0] tempoversampling;
		assign tempoversampling = 5'b01111;
	wire [7:0] tempdatalength;
	assign tempdatalength = 3'b111;

reg [2:0]	state  = `RESET;
reg [7:0]	data   = 8'b0; // temporary data
reg [2:0]	dataindex = 3'b0; // for 8-bit data
reg 		txparitybitout = 1'b0;

// counting 16 ticks and send data 
reg internalclk;
reg [4:0] clkindex = 5'b0;
initial clkindex <= 5'b0;
always @(posedge TICK) begin
	if (clkindex == tempoversampling) begin
		clkindex <= 5'b0;
		internalclk <= 1'b1;
	end
	else begin
		clkindex <= clkindex + 1'b1;
		internalclk <= 1'b0;
	end
end

/*always @(posedge CLK) begin
	if (RESETN) begin
		DOUT 		<= 1'b1; 
		BUSY		<= 1'b0;
		DONE 		<= 1'b1;
		dataindex 	<= 3'b0;
		data    	<= 8'b0;
		txparitybitout 	<= 1'b0;
	end
end*/

initial begin
	DOUT 		<= 1'b1; 
	DONE 		<= 1'b1;
	BUSY		<= 1'b0;
	dataindex 	<= 3'b0;
	data    	<= 8'b0;
	txparitybitout 	<= 1'b0;
end
// the main State machine
always @(posedge internalclk or posedge RESETN) begin
	if(RESETN) begin
		DOUT 		<= 1'b1; 
		BUSY		<= 1'b0;
		DONE 		<= 1'b1;
		dataindex 	<= 3'b0;
		data    	<= 8'b0;
		txparitybitout 	<= 1'b0;
	end
	else begin
	case (state)
		default : begin
			state <= `IDLE;
			if (START) begin // THIS WAS (START & EN)
                data    <= DIN; // save a copy of input data
				DONE 	<= 1'b1;
				BUSY	<= 1'b0;
                state   <= `START_BIT;
            end
		end
		`IDLE : begin // 2 - reset everything to 0 and put high the tx line
			DOUT 		<= 1'b1; 
			DONE 		<= 1'b1;
			BUSY		<= 1'b0;
			dataindex 	<= 3'b0;
			data    	<= 8'b0;
			if (START) begin  // THIS WAS (START & EN)
                state   <= `START_BIT;
            end
		end
		`START_BIT : begin // 3 - 
			data    	<= DIN; // save a copy of input data
			DOUT 		<= 1'b0; //send start bit low "0"
			DONE 		<= 1'b0;
			BUSY		<= 1'b1;
			state		<= `DATA_BITS; //change state to data frame
		end
		`DATA_BITS : begin // 4 -
			DOUT 		<= data[dataindex];
			if (dataindex==tempdatalength) begin
				dataindex	<= 3'b0;
				// even parity
				txparitybitout 	<= ^data; 
				state 		<= `PARITY_BIT;
			end
			else
				dataindex 	<= dataindex + 1'b1;
		end
		`PARITY_BIT : begin
			DOUT 		<= txparitybitout; //send parity bit
			state		<= `STOP_BIT; //change state to data frame
		end
		`STOP_BIT : begin
			DOUT 	<= 1'b1;
			DONE	<= 1'b1;
			BUSY	<= 1'b0;
			txparitybitout <= 1'b0;
			state	<= `IDLE;
		end
	endcase
	end
end
endmodule

//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//-------------------------------RX-------------------------------------

module MS_UART_RX(
	input wire CLK,
	input wire RESETN,
	input wire DIN,
	input wire TICK,
	
	output reg [7:0] DOUT,
	output reg DONE,
	output reg ERR);
	
	wire [4:0] tempoversampling;
		assign tempoversampling = 5'b01111;
	wire [7:0] tempdatalength;
		assign tempdatalength = 8'b00001000;
	
	// states of state machine
    reg [2:0] RESET 	= 3'b000;
    reg [2:0] IDLE 	= 3'b001;
    reg [2:0] DATA_BITS = 3'b010;
    reg [2:0] STOP_BIT 	= 3'b011;
    reg [2:0] PARITY_BIT= 3'b100;
	
	reg [2:0] state;
	reg [3:0] bitindex 	= 4'b0; //index for data
	reg [1:0] shift 	= 2'b0;//reg for input signal state
	reg [3:0] counter 	= 5'b0; // counter for 16 oversampling
	reg [7:0] rdata 	= 8'b0; //receiveed data
	reg EN;
	reg		  rxparitybitin = 1'b0; //temp parity bit
	reg 	  rxparitybitcal;//,rxparitybitcal2,rxparitybitcal3;
	reg [7:0] tempdataholder;
	
initial begin
	DOUT 		<= 8'b0; 
	DONE 		<= 1'b0;
	ERR 		<= 1'b0;
	EN			<= 1'b1;
	rxparitybitin 	<= 1'b0;
end

/*always @(posedge CLK) begin
	if (RESETN) begin
		state <= RESET;
	end
end*/

always @(posedge TICK or posedge RESETN) begin
	if (RESETN) begin
		state <= RESET;
	end
	else begin
	if (!EN) state <= RESET;
	
	case (state)
		RESET : begin 
			DONE 			<= 1'b0;
			//BUSY 			<= 1'b0;
			ERR 			<= 1'b0;
			bitindex 		<= 4'b0;
			counter 		<= 5'b0;
			rdata			<= 8'b0;
			rxparitybitin 		<= 1'b0;
			if (EN)	state 	<= IDLE;
		end
		IDLE : begin
			if (counter==(tempoversampling-4'b1000)) begin
				ERR 		<= 1'b0;
				bitindex 	<= 4'b0;
				counter 	<= 5'b0;
				rdata		<= 8'b0;
				rxparitybitin 	<= 1'b0;
				state 		<= DATA_BITS;
			end
			else if (!DIN) begin
				counter <= counter + 1'b1;
			end
			else begin
				counter <= counter;
			end
		end
		DATA_BITS : begin // state 2
			if (counter==tempoversampling) begin
				rdata[bitindex] <= DIN;
				bitindex 		<= bitindex + 1'b1;
				counter			<= 5'b0;
			end
			else begin
				counter 		<= counter + 1'b1;
			end
			
			if(bitindex== tempdatalength) begin
				counter 	<= 5'b0;
				bitindex 	<= 4'b0;
				DOUT		<= rdata;
				tempdataholder <= rdata;
				state		<= PARITY_BIT;
			end
		end
		PARITY_BIT : begin		//state 4	
			if (counter==tempoversampling) begin 
				counter 		<= 5'b0;
				rxparitybitin	<= DIN;
				state 			<= STOP_BIT;
			end
			else begin
				counter <= counter + 1'b1;
			end
		end
		STOP_BIT : begin	 //state 3
			if (counter==tempoversampling) begin
					rxparitybitcal	= ^rdata;
					if (rxparitybitin !== rxparitybitcal) begin
						ERR 	<= 1'b1;
					end
				  else begin
					  ERR 	<= 1'b0;
				  end
				DONE 		<= 1'b1;
				bitindex 	<= 4'b0;
				counter 	<= 5'b0;
				rdata		<= 8'b0;
				state 		<= RESET;
			end
			else begin
				counter <= counter + 1'b1;
			end
		end
		
		default : begin
			DOUT 		<= 8'b0; 
			DONE 		<= 1'b0;	
			ERR 		<= 1'b0;
			bitindex 	<= 1'b0;
			counter 	<= 5'b0;
			rdata		<= 8'b0;
			if (EN)	state <= IDLE;
		end
	endcase
	end
end
endmodule

//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//------------------------------BAUDRATE--------------------------------

module MS_UART_BAUDGEN(
  input CLK,   		 // board clock
  input RESETN,
  output reg BAUDTICK // baud rate for rx
);

reg [15:0] UBRR = 16'h0007;
reg [16 - 1:0] Counter = 0;

initial begin
    BAUDTICK = 1'b0;
end

always @(posedge CLK) begin
	if (RESETN) begin
	BAUDTICK = 1'b0;
	end 
	else begin
		if (Counter == UBRR) begin
			Counter <= 0;
			BAUDTICK <= 1'b1; //~BAUDTICK;
		end 
		else begin
			Counter <= Counter + 1'b1;
			if (Counter==2) BAUDTICK <= 1'b0;
		end
	end
end
endmodule

module MSUART(
  input  [7:0] io_in, 
  output  [7:0] io_out
  );
 
  //io_in  => datain4,datain3,datain2,datain1,rx,send,reset,clk
  //io_out => non,non,busy,tx,dataout4,dataout3,dataout2,dataout1
  
  wire ms_clk;
  wire ms_reset;
  wire ms_send;
  reg [3:0] ms_datain;
  wire [7:0] ms_dataout;
  wire tb_tick2;
  wire ms_rx;
  wire ms_tx;
  
  assign ms_clk = io_in[0];
  assign ms_reset = io_in[1];
  assign ms_send = io_in[2];
  assign ms_rx = io_in[3];
  //assign ms_datain = io_in[7:4];
  
  assign io_out = {2'b0, tb_busytx, ms_tx, ms_dataout[3:0]};
  /*assign io_out[3:0] = ms_dataout[3:0]; 
  assign io_out[4] = ms_tx;
  assign io_out[5] = tb_busytx;*/
    
  wire tb_donetx, tb_busytx;
  wire tb_donerx, tb_errrx;
  reg ms_start;
	
	always @(posedge tb_tick2) begin
		if(ms_send & !tb_busytx) begin
			ms_datain <=  io_in[7:4];
	    		ms_start <= 1'b1;
		end
		if (tb_busytx) begin
			ms_start <= 1'b0;
		end
	end
	
	/*always@(posedge tb_busytx) begin
		ms_start <= 1'b0;
	end*/
	

  // TX MODULE
	MS_UART_TX DUT_TX(
	.CLK(ms_clk),		
	.RESETN(ms_reset),
	.START(ms_start),
	.TICK(tb_tick2),
	.DIN(ms_datain),
	
	.DONE(tb_donetx),
	.BUSY(tb_busytx),
	.DOUT(ms_tx));
	
	// RX MODULE
  MS_UART_RX DUT_RX(
	.CLK(ms_clk),
	.RESETN(ms_reset),
	.DIN(ms_rx),
	.TICK(tb_tick2),
	
	.DOUT(ms_dataout),
	.DONE(tb_donerx),
	.ERR(tb_errrx));
	
	MS_UART_BAUDGEN DUT_BAUDGEN(
  .CLK(ms_clk),   		 // board clock
  .RESETN(ms_reset),
  .BAUDTICK(tb_tick2) // baud rate for rx
  );
endmodule
