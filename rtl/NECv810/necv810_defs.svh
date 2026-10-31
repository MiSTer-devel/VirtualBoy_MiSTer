// NEC V810 decode tables, constants and pure datapath functions shared by
// the pipeline. Included inside NECv810.

	localparam [1:0] SIZE_B = 2'b00;
	localparam [1:0] SIZE_H = 2'b01;
	localparam [1:0] SIZE_W = 2'b10;
	localparam [4:0] SYS_EIPC  = 5'd0;
	localparam [4:0] SYS_EIPSW = 5'd1;
	localparam [4:0] SYS_FEPC  = 5'd2;
	localparam [4:0] SYS_FEPSW = 5'd3;
	localparam [4:0] SYS_ECR   = 5'd4;
	localparam [4:0] SYS_PSW   = 5'd5;
	localparam [4:0] SYS_PIR   = 5'd6;
	localparam [4:0] SYS_TKCW  = 5'd7;
	localparam [4:0] SYS_CHCW  = 5'd24;
	localparam [4:0] SYS_ADTRE = 5'd25;
	localparam [4:0] SYS_SR29  = 5'd29;
	localparam [4:0] SYS_SR30  = 5'd30;
	localparam [4:0] SYS_SR31  = 5'd31;
	localparam [31:0] PIR_VALUE      = VB_EXT ? 32'h0000_5346 : 32'h0000_8100;
	localparam [31:0] TKCW_VALUE     = 32'h0000_00e0;
	localparam [31:0] SR30_VALUE     = 32'h0000_0004;
	localparam [31:0] PSW_WRITE_MASK = 32'h000f_f3ff;
	localparam [31:0] PSW_Z_MASK = 32'h0000_0001;
	localparam [31:0] PSW_ID_MASK = 32'h0000_1000;
	localparam [31:0] PSW_AE_MASK = 32'h0000_2000;
	localparam [31:0] PSW_EP_MASK = 32'h0000_4000;
	localparam [31:0] PSW_NP_MASK = 32'h0000_8000;
	localparam [15:0] EXC_CODE_ILLEGAL  = 16'hff90;
	localparam [15:0] EXC_CODE_ZERODIV  = 16'hff80;
	localparam [15:0] EXC_CODE_ADDRTRAP = 16'hffc0;
	localparam [15:0] EXC_CODE_NMI      = 16'hffd0;
	localparam [5:0] OP_MOV_R  = 6'b000000;
	localparam [5:0] OP_ADD_R  = 6'b000001;
	localparam [5:0] OP_SUB_R  = 6'b000010;
	localparam [5:0] OP_CMP_R  = 6'b000011;
	localparam [5:0] OP_SHL_R  = 6'b000100;
	localparam [5:0] OP_SHR_R  = 6'b000101;
	localparam [5:0] OP_JMP    = 6'b000110;
	localparam [5:0] OP_SAR_R  = 6'b000111;
	localparam [5:0] OP_MUL    = 6'b001000;
	localparam [5:0] OP_DIV    = 6'b001001;
	localparam [5:0] OP_MULU   = 6'b001010;
	localparam [5:0] OP_DIVU   = 6'b001011;
	localparam [5:0] OP_OR_R   = 6'b001100;
	localparam [5:0] OP_AND_R  = 6'b001101;
	localparam [5:0] OP_XOR_R  = 6'b001110;
	localparam [5:0] OP_NOT_R  = 6'b001111;
	localparam [5:0] OP_MOV_I  = 6'b010000;
	localparam [5:0] OP_ADD_I  = 6'b010001;
	localparam [5:0] OP_SETF   = 6'b010010;
	localparam [5:0] OP_CMP_I  = 6'b010011;
	localparam [5:0] OP_SHL_I  = 6'b010100;
	localparam [5:0] OP_SHR_I  = 6'b010101;
	localparam [5:0] OP_CLI    = 6'b010110;
	localparam [5:0] OP_TRAP   = 6'b011000;
	localparam [5:0] OP_RETI   = 6'b011001;
	localparam [5:0] OP_HALT   = 6'b011010;
	localparam [5:0] OP_LDSR   = 6'b011100;
	localparam [5:0] OP_STSR   = 6'b011101;
	localparam [5:0] OP_SEI    = 6'b011110;
	localparam [5:0] OP_SAR_I  = 6'b010111;
	localparam [5:0] OP_MOVEA  = 6'b101000;
	localparam [5:0] OP_ADDI   = 6'b101001;
	localparam [5:0] OP_JR     = 6'b101010;
	localparam [5:0] OP_JAL    = 6'b101011;
	localparam [5:0] OP_ORI    = 6'b101100;
	localparam [5:0] OP_ANDI   = 6'b101101;
	localparam [5:0] OP_XORI   = 6'b101110;
	localparam [5:0] OP_MOVHI  = 6'b101111;
	localparam [5:0] OP_LD_B   = 6'b110000;
	localparam [5:0] OP_LD_H   = 6'b110001;
	localparam [5:0] OP_LD_W   = 6'b110011;
	localparam [5:0] OP_ST_B   = 6'b110100;
	localparam [5:0] OP_ST_H   = 6'b110101;
	localparam [5:0] OP_ST_W   = 6'b110111;
	localparam [5:0] OP_IN_B   = 6'b111000;
	localparam [5:0] OP_IN_H   = 6'b111001;
	localparam [5:0] OP_CAXI   = 6'b111010;
	localparam [5:0] OP_IN_W   = 6'b111011;
	localparam [5:0] OP_OUT_B  = 6'b111100;
	localparam [5:0] OP_OUT_H  = 6'b111101;
	localparam [5:0] OP_EXT    = 6'b111110;
	localparam [5:0] OP_BITSTR = 6'b011111;
	localparam [5:0] OP_OUT_W  = 6'b111111;
	localparam [5:0] SUBOP_CMPF  = 6'b000000;
	localparam [5:0] SUBOP_CVTWS = 6'b000010;
	localparam [5:0] SUBOP_CVTSW = 6'b000011;
	localparam [5:0] SUBOP_ADDF  = 6'b000100;
	localparam [5:0] SUBOP_SUBF  = 6'b000101;
	localparam [5:0] SUBOP_MULF  = 6'b000110;
	localparam [5:0] SUBOP_DIVF  = 6'b000111;
	localparam [5:0] SUBOP_XB    = 6'b001000;
	localparam [5:0] SUBOP_XH    = 6'b001001;
	localparam [5:0] SUBOP_REV   = 6'b001010;
	localparam [5:0] SUBOP_TRNCSW = 6'b001011;
	localparam [5:0] SUBOP_MPYHW = 6'b001100;
	localparam [4:0] SUBOP_SCH0BSU = 5'b00000;
	localparam [4:0] SUBOP_SCH0BSD = 5'b00001;
	localparam [4:0] SUBOP_SCH1BSU = 5'b00010;
	localparam [4:0] SUBOP_SCH1BSD = 5'b00011;
	localparam [4:0] SUBOP_ORBSU   = 5'b01000;
	localparam [4:0] SUBOP_ANDBSU  = 5'b01001;
	localparam [4:0] SUBOP_XORBSU  = 5'b01010;
	localparam [4:0] SUBOP_MOVBSU  = 5'b01011;
	localparam [4:0] SUBOP_ORNBSU  = 5'b01100;
	localparam [4:0] SUBOP_ANDNBSU = 5'b01101;
	localparam [4:0] SUBOP_XORNBSU = 5'b01110;
	localparam [4:0] SUBOP_NOTBSU  = 5'b01111;
	localparam [2:0] OP_BCOND  = 3'b100;
	localparam [3:0] MK_NOP    = 4'd0;
	localparam [3:0] MK_ALU    = 4'd1;
	localparam [3:0] MK_BRANCH = 4'd2;
	localparam [3:0] MK_MEM    = 4'd3;
	localparam [3:0] MK_LDSR   = 4'd4;
	localparam [3:0] MK_STSR   = 4'd5;
	localparam [3:0] MK_HALT   = 4'd6;
	localparam [3:0] MK_TRAP   = 4'd7;
	localparam [3:0] MK_RETI   = 4'd8;
	localparam [3:0] MK_LONG   = 4'd9;
	localparam [3:0] MK_CAXI   = 4'd10;
	localparam [3:0] MK_BITSTR = 4'd11;
	localparam [3:0] MK_ILL    = 4'd15;
	localparam [3:0] ALU_PASS_A = 4'd0;
	localparam [3:0] ALU_PASS_B = 4'd1;
	localparam [3:0] ALU_ADD    = 4'd2;
	localparam [3:0] ALU_SUB    = 4'd3;
	localparam [3:0] ALU_OR     = 4'd4;
	localparam [3:0] ALU_AND    = 4'd5;
	localparam [3:0] ALU_XOR    = 4'd6;
	localparam [3:0] ALU_NOT    = 4'd7;
	localparam [3:0] ALU_SHL    = 4'd8;
	localparam [3:0] ALU_SHR    = 4'd9;
	localparam [3:0] ALU_SAR    = 4'd10;
	localparam [3:0] ALU_SETF   = 4'd11;
	localparam [3:0] LONG_CLI   = 4'd0;
	localparam [3:0] LONG_SEI   = 4'd1;
	localparam [3:0] LONG_XB    = 4'd2;
	localparam [3:0] LONG_XH    = 4'd3;
	localparam [3:0] LONG_REV   = 4'd4;
	localparam [3:0] LONG_MPYHW = 4'd5;
	localparam [3:0] LONG_CMPF  = 4'd6;
	localparam [3:0] LONG_CVTWS = 4'd7;
	localparam [3:0] LONG_CVTSW = 4'd8;
	localparam [3:0] LONG_TRNCSW = 4'd9;
	localparam [3:0] LONG_ADDF  = 4'd10;
	localparam [3:0] LONG_SUBF  = 4'd11;
	localparam [3:0] LONG_MUL   = 4'd12;
	localparam [3:0] LONG_DIV   = 4'd13;
	localparam [3:0] LONG_MULU  = 4'd14;
	localparam [3:0] LONG_DIVU  = 4'd15;
	localparam [1:0] SA_ZERO = 2'd0;
	localparam [1:0] SA_REG1 = 2'd1;
	localparam [1:0] SA_REG2 = 2'd2;
	localparam [3:0] SB_ZERO   = 4'd0;
	localparam [3:0] SB_REG1   = 4'd1;
	localparam [3:0] SB_REG2   = 4'd2;
	localparam [3:0] SB_IMM5S  = 4'd3;
	localparam [3:0] SB_IMM16S = 4'd4;
	localparam [3:0] SB_IMM16Z = 4'd5;
	localparam [3:0] SB_IMM16H = 4'd6;
	localparam [3:0] SB_IMM5Z  = 4'd8;
	localparam [5:0] UADDR_NOP    = 6'd0;
	localparam [5:0] UADDR_MOV_R  = 6'd1;
	localparam [5:0] UADDR_ADD_R  = 6'd2;
	localparam [5:0] UADDR_SUB_R  = 6'd3;
	localparam [5:0] UADDR_CMP_R  = 6'd4;
	localparam [5:0] UADDR_SHL_R  = 6'd5;
	localparam [5:0] UADDR_SHR_R  = 6'd6;
	localparam [5:0] UADDR_JMP    = 6'd7;
	localparam [5:0] UADDR_SAR_R  = 6'd8;
	localparam [5:0] UADDR_OR_R   = 6'd9;
	localparam [5:0] UADDR_AND_R  = 6'd10;
	localparam [5:0] UADDR_XOR_R  = 6'd11;
	localparam [5:0] UADDR_NOT_R  = 6'd12;
	localparam [5:0] UADDR_MOV_I  = 6'd13;
	localparam [5:0] UADDR_ADD_I  = 6'd14;
	localparam [5:0] UADDR_SETF   = 6'd15;
	localparam [5:0] UADDR_CMP_I  = 6'd16;
	localparam [5:0] UADDR_SHL_I  = 6'd17;
	localparam [5:0] UADDR_SHR_I  = 6'd18;
	localparam [5:0] UADDR_SAR_I  = 6'd19;
	localparam [5:0] UADDR_HALT   = 6'd20;
	localparam [5:0] UADDR_LDSR   = 6'd21;
	localparam [5:0] UADDR_STSR   = 6'd22;
	localparam [5:0] UADDR_BCOND  = 6'd23;
	localparam [5:0] UADDR_MOVEA  = 6'd24;
	localparam [5:0] UADDR_ADDI   = 6'd25;
	localparam [5:0] UADDR_JR     = 6'd26;
	localparam [5:0] UADDR_JAL    = 6'd27;
	localparam [5:0] UADDR_ORI    = 6'd28;
	localparam [5:0] UADDR_ANDI   = 6'd29;
	localparam [5:0] UADDR_XORI   = 6'd30;
	localparam [5:0] UADDR_MOVHI  = 6'd31;
	localparam [5:0] UADDR_MEM_LD_B  = 6'd32;
	localparam [5:0] UADDR_MEM_LD_H  = 6'd33;
	localparam [5:0] UADDR_MEM_LD_W  = 6'd34;
	localparam [5:0] UADDR_MEM_ST_B  = 6'd35;
	localparam [5:0] UADDR_MEM_ST_H  = 6'd36;
	localparam [5:0] UADDR_MEM_ST_W  = 6'd37;
	localparam [5:0] UADDR_MEM_IN_B  = 6'd38;
	localparam [5:0] UADDR_MEM_IN_H  = 6'd39;
	localparam [5:0] UADDR_MEM_IN_W  = 6'd40;
	localparam [5:0] UADDR_MEM_OUT_B = 6'd41;
	localparam [5:0] UADDR_MEM_OUT_H = 6'd42;
	localparam [5:0] UADDR_MEM_OUT_W = 6'd43;
	localparam [5:0] UADDR_TRAP   = 6'd44;
	localparam [5:0] UADDR_RETI   = 6'd45;
	localparam [5:0] UADDR_MUL    = 6'd46;
	localparam [5:0] UADDR_DIV    = 6'd47;
	localparam [5:0] UADDR_MULU   = 6'd48;
	localparam [5:0] UADDR_DIVU   = 6'd49;
	localparam [5:0] UADDR_CLI    = 6'd50;
	localparam [5:0] UADDR_SEI    = 6'd51;
	localparam [5:0] UADDR_XB     = 6'd52;
	localparam [5:0] UADDR_XH     = 6'd53;
	localparam [5:0] UADDR_REV    = 6'd54;
	localparam [5:0] UADDR_MPYHW  = 6'd55;
	localparam [5:0] UADDR_CAXI   = 6'd56;
	localparam [5:0] UADDR_BITSTR = 6'd57;
	localparam [5:0] UADDR_CMPF   = 6'd58;
	localparam [5:0] UADDR_CVTWS  = 6'd59;
	localparam [5:0] UADDR_CVTSW  = 6'd60;
	localparam [5:0] UADDR_FPU    = 6'd61;
	localparam [5:0] UADDR_ILL    = 6'd62;
	localparam [2:0] BS_OP_OR    = 3'd0;
	localparam [2:0] BS_OP_AND   = 3'd1;
	localparam [2:0] BS_OP_XOR   = 3'd2;
	localparam [2:0] BS_OP_MOV   = 3'd3;
	localparam [2:0] BS_OP_ORN   = 3'd4;
	localparam [2:0] BS_OP_ANDN  = 3'd5;
	localparam [2:0] BS_OP_XORN  = 3'd6;
	localparam [2:0] BS_OP_NOT   = 3'd7;
	localparam [1:0] ASYNC_EXC_NONE = 2'd0;
	localparam [1:0] ASYNC_EXC_ADDR = 2'd1;
	localparam [1:0] ASYNC_EXC_NMI  = 2'd2;
	localparam [1:0] ASYNC_EXC_IRQ  = 2'd3;

	function [31:0] ucode_fn;
		input [5:0] ue;
		begin
			case (ue)
				6'd0: ucode_fn = pack_ucode_fn(MK_NOP,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // NOP
				6'd1: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_PASS_A, SA_REG1, SB_ZERO,   1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MOV r1, r2
				6'd2: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_ADD,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // ADD r1, r2
				6'd3: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SUB,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SUB r1, r2
				6'd4: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SUB,    SA_REG2, SB_REG1,   1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // CMP r1, r2
				6'd5: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SHL,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SHL r1, r2
				6'd6: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SHR,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SHR r1, r2
				6'd7: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // JMP [r1]
				6'd8: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SAR,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SAR r1, r2
				6'd9: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_OR,     SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // OR r1, r2
				6'd10: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_AND,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // AND r1, r2
				6'd11: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_XOR,    SA_REG2, SB_REG1,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // XOR r1, r2
				6'd12: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_NOT,    SA_REG1, SB_ZERO,   1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // NOT r1, r2
				6'd13: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_PASS_B, SA_ZERO, SB_IMM5S,  1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MOV imm5, r2
				6'd14: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_ADD,    SA_REG2, SB_IMM5S,  1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // ADD imm5, r2
				6'd15: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SETF,   SA_ZERO, SB_ZERO,   1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SETF cond, r2
				6'd16: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SUB,    SA_REG2, SB_IMM5S,  1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // CMP imm5, r2
				6'd17: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SHL,    SA_REG2, SB_IMM5Z,  1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SHL imm5, r2
				6'd18: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SHR,    SA_REG2, SB_IMM5Z,  1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SHR imm5, r2
				6'd19: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_SAR,    SA_REG2, SB_IMM5Z,  1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SAR imm5, r2
				6'd20: ucode_fn = pack_ucode_fn(MK_HALT,   ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // HALT
				6'd21: ucode_fn = pack_ucode_fn(MK_LDSR,   ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // LDSR
				6'd22: ucode_fn = pack_ucode_fn(MK_STSR,   ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // STSR
				6'd23: ucode_fn = pack_ucode_fn(MK_BRANCH, ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // Bcond
				6'd24: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_ADD,    SA_REG1, SB_IMM16S, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MOVEA
				6'd25: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_ADD,    SA_REG1, SB_IMM16S, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // ADDI
				6'd26: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // JR
				6'd27: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // JAL
				6'd28: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_OR,     SA_REG1, SB_IMM16Z, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // ORI
				6'd29: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_AND,    SA_REG1, SB_IMM16Z, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // ANDI
				6'd30: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_XOR,    SA_REG1, SB_IMM16Z, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // XORI
				6'd31: ucode_fn = pack_ucode_fn(MK_ALU,    ALU_ADD,    SA_REG1, SB_IMM16H, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MOVHI
				6'd32: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_B, 1'b1, 1'b1, 6'd0,  1'b0); // LD.B
				6'd33: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b1, 1'b1, 6'd0,  1'b0); // LD.H
				6'd34: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_W, 1'b0, 1'b1, 6'd0,  1'b0); // LD.W
				6'd35: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, SIZE_B, 1'b0, 1'b1, 6'd0,  1'b0); // ST.B
				6'd36: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // ST.H
				6'd37: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, SIZE_W, 1'b0, 1'b1, 6'd0,  1'b0); // ST.W
				6'd38: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, SIZE_B, 1'b0, 1'b1, 6'd0,  1'b0); // IN.B
				6'd39: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // IN.H
				6'd40: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, SIZE_W, 1'b0, 1'b1, 6'd0,  1'b0); // IN.W
				6'd41: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, SIZE_B, 1'b0, 1'b1, 6'd0,  1'b0); // OUT.B
				6'd42: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // OUT.H
				6'd43: ucode_fn = pack_ucode_fn(MK_MEM,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, SIZE_W, 1'b0, 1'b1, 6'd0,  1'b0); // OUT.W
				6'd44: ucode_fn = pack_ucode_fn(MK_TRAP,   ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // TRAP
				6'd45: ucode_fn = pack_ucode_fn(MK_RETI,   ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // RETI
				6'd46: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_MUL,   SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MUL
				6'd47: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_DIV,   SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b1); // DIV
				6'd48: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_MULU,  SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MULU
				6'd49: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_DIVU,  SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b1); // DIVU
				6'd50: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_CLI,   SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // CLI
				6'd51: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_SEI,   SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // SEI
				6'd52: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_XB,    SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // XB
				6'd53: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_XH,    SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // XH
				6'd54: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_REV,   SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // REV
				6'd55: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_MPYHW, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // MPYHW
				6'd56: ucode_fn = pack_ucode_fn(MK_CAXI,   ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_W, 1'b0, 1'b1, 6'd0,  1'b0); // CAXI
				6'd57: ucode_fn = pack_ucode_fn(MK_BITSTR, ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_W, 1'b0, 1'b1, 6'd0,  1'b0); // Bit-string family
				6'd58: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_CMPF,  SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b1); // CMPF.S
				6'd59: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_CVTWS, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b1); // CVT.WS
				6'd60: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_CVTSW, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b1); // CVT.SW / TRNC.SW shared entry
				6'd61: ucode_fn = pack_ucode_fn(MK_LONG,   LONG_ADDF,  SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b1); // ADDF.S / SUBF.S / MULF.S / DIVF.S shared entry
				6'd62: ucode_fn = pack_ucode_fn(MK_ILL,    ALU_PASS_A, SA_ZERO, SB_ZERO,   1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0,  1'b0); // Illegal-op sentinel.
				default: ucode_fn = pack_ucode_fn(MK_ILL, ALU_PASS_A, SA_ZERO, SB_ZERO, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, SIZE_H, 1'b0, 1'b1, 6'd0, 1'b0);
			endcase
		end
	endfunction

	function [31:0] pack_ucode_fn;
		input [3:0] kind;
		input [3:0] alu_op;
		input [1:0] srca;
		input [3:0] srcb;
		input       write_gpr;
		input       write_psw;
		input       write_link;
		input       pc_from_result;
		input       pc_from_bcond;
		input       mem_write;
		input       mem_io;
		input [1:0] mem_size;
		input       mem_signext;
		input       end_uop;
		input [5:0] next_addr;
		input       abortable;
		begin
			pack_ucode_fn = 32'd0;
			pack_ucode_fn[3:0] = kind;
			pack_ucode_fn[7:4] = alu_op;
			pack_ucode_fn[9:8] = srca;
			pack_ucode_fn[13:10] = srcb;
			pack_ucode_fn[14] = write_gpr;
			pack_ucode_fn[15] = write_psw;
			pack_ucode_fn[16] = write_link;
			pack_ucode_fn[17] = pc_from_result;
			pack_ucode_fn[18] = pc_from_bcond;
			pack_ucode_fn[19] = mem_write;
			pack_ucode_fn[20] = mem_io;
			pack_ucode_fn[22:21] = mem_size;
			pack_ucode_fn[23] = mem_signext;
			pack_ucode_fn[24] = end_uop;
			pack_ucode_fn[30:25] = next_addr;
			pack_ucode_fn[31] = abortable;
		end
	endfunction

	function instr_needs_second_half_fn;
		input [15:0] iw;
		begin
			instr_needs_second_half_fn = (iw[15:13] == 3'b101) || (iw[15:13] == 3'b110) || (iw[15:13] == 3'b111);
		end
	endfunction

	function [31:0] mem_read_data_fn;
		input [1:0]  size;
		input        signext;
		input [31:0] data;
		begin
			case (size)
				SIZE_B: mem_read_data_fn = signext ?
					{{24{data[7]}}, data[7:0]} : {24'd0, data[7:0]};
				SIZE_W: mem_read_data_fn = data;
				default: mem_read_data_fn = signext ?
					{{16{data[15]}}, data[15:0]} : {16'd0, data[15:0]};
			endcase
		end
	endfunction

	function address_error_fn;
		input [31:0] addr;
		input [1:0]  size;
		begin
			case (size)
				SIZE_H: address_error_fn = addr[0];
				SIZE_W: address_error_fn = |addr[1:0];
				default: address_error_fn = 1'b0;
			endcase
		end
	endfunction

	function chcw_invalid_combination_fn;
		input [31:0] command;
		begin
			// ICE with ICC clears and enables together. Other combined
			// maintenance commands are rejected.
			chcw_invalid_combination_fn =
				(command[1] && (command[4] || command[5])) ||
				(command[0] && command[4]) ||
				(command[0] && command[5]) ||
				(command[4] && command[5]);
		end
	endfunction

	function [31:0] align_addr_fn;
		input [31:0] addr;
		input [1:0]  size;
		begin
			case (size)
				SIZE_H: align_addr_fn = {addr[31:1], 1'b0};
				SIZE_W: align_addr_fn = {addr[31:2], 2'b00};
				default: align_addr_fn = addr;
			endcase
		end
	endfunction

	function [33:0] alu_addsub_fn;
		input [31:0] lhs;
		input [31:0] rhs;
		input subtract;
		reg [32:0] sum_v;
		begin
			sum_v = {1'b0, lhs} +
				{1'b0, (rhs ^ {32{subtract}})} +
				{32'd0, subtract};
			alu_addsub_fn[31:0] = sum_v[31:0];
			alu_addsub_fn[32] = subtract ? ~sum_v[32] : sum_v[32];
			alu_addsub_fn[33] =
				(~(lhs[31] ^ (rhs[31] ^ subtract))) &
				(lhs[31] ^ sum_v[31]);
		end
	endfunction

	function [31:0] reverse32_fn;
		input [31:0] value;
		begin
			reverse32_fn = {
				value[0],  value[1],  value[2],  value[3],
				value[4],  value[5],  value[6],  value[7],
				value[8],  value[9],  value[10], value[11],
				value[12], value[13], value[14], value[15],
				value[16], value[17], value[18], value[19],
				value[20], value[21], value[22], value[23],
				value[24], value[25], value[26], value[27],
				value[28], value[29], value[30], value[31]
			};
		end
	endfunction

	function [32:0] barrel_shift_fn;
		input [31:0] value;
		input [4:0] amount;
		input left;
		input arithmetic;
		reg fill_v;
		reg [32:0] stage0_v;
		reg [32:0] stage1_v;
		reg [32:0] stage2_v;
		reg [32:0] stage4_v;
		reg [32:0] stage8_v;
		reg [32:0] stage16_v;
		reg [31:0] ordered_result_v;
		begin
			fill_v = arithmetic & value[31];
			stage0_v = left ? {reverse32_fn(value), 1'b0} : {value, 1'b0};
			stage1_v = amount[0] ? {fill_v, stage0_v[32:1]} : stage0_v;
			stage2_v = amount[1] ? {{2{fill_v}}, stage1_v[32:2]} : stage1_v;
			stage4_v = amount[2] ? {{4{fill_v}}, stage2_v[32:4]} : stage2_v;
			stage8_v = amount[3] ? {{8{fill_v}}, stage4_v[32:8]} : stage4_v;
			stage16_v = amount[4] ? {{16{fill_v}}, stage8_v[32:16]} : stage8_v;
			ordered_result_v = stage16_v[32:1];
			barrel_shift_fn[31:0] = left ?
				reverse32_fn(ordered_result_v) : ordered_result_v;
			barrel_shift_fn[32] = stage16_v[0];
		end
	endfunction

	function [31:0] sx5_fn;
		input [4:0] imm5;
		begin
			sx5_fn = {{27{imm5[4]}}, imm5};
		end
	endfunction

	function [31:0] sx9_fn;
		input [8:0] imm9;
		begin
			sx9_fn = {{23{imm9[8]}}, imm9};
		end
	endfunction

	function [31:0] sx16_fn;
		input [15:0] imm16;
		begin
			sx16_fn = {{16{imm16[15]}}, imm16};
		end
	endfunction

	function [31:0] sx26_fn;
		input [25:0] imm26;
		begin
			sx26_fn = {{6{imm26[25]}}, imm26};
		end
	endfunction

	function [31:0] issue_immediate_fn;
		input [5:0]  uentry;
		input [31:0] iw;
		begin
			case (uentry)
				UADDR_MOV_I,
				UADDR_ADD_I,
				UADDR_CMP_I: issue_immediate_fn = sx5_fn(iw[20:16]);

				UADDR_SHL_I,
				UADDR_SHR_I,
				UADDR_SAR_I: issue_immediate_fn = {27'd0, iw[20:16]};

				UADDR_MOVEA,
				UADDR_ADDI,
				UADDR_MEM_LD_B,
				UADDR_MEM_LD_H,
				UADDR_MEM_LD_W,
				UADDR_MEM_ST_B,
				UADDR_MEM_ST_H,
				UADDR_MEM_ST_W,
				UADDR_MEM_IN_B,
				UADDR_MEM_IN_H,
				UADDR_MEM_IN_W,
				UADDR_MEM_OUT_B,
				UADDR_MEM_OUT_H,
				UADDR_MEM_OUT_W,
				UADDR_CAXI: issue_immediate_fn = sx16_fn(iw[15:0]);

				UADDR_ORI,
				UADDR_ANDI,
				UADDR_XORI: issue_immediate_fn = {16'd0, iw[15:0]};

				UADDR_MOVHI: issue_immediate_fn = {iw[15:0], 16'd0};

				UADDR_JR,
				UADDR_JAL: issue_immediate_fn = sx26_fn(iw[25:0]);

				UADDR_BCOND: issue_immediate_fn = sx9_fn(iw[24:16]);

				default: issue_immediate_fn = 32'd0;
			endcase
		end
	endfunction

	function [31:0] chcw_readback_fn;
		input cache_enable;
		begin
			// CHCW reads back only ICE; all other fields are zero.
			chcw_readback_fn = {30'd0, cache_enable, 1'b0};
		end
	endfunction

	function [31:0] sysreg_read_fn;
		input [4:0] regid;
		input [31:0] eipc;
		input [31:0] eipsw;
		input [31:0] fepc;
		input [31:0] fepsw;
		input [31:0] ecr;
		input [31:0] psw;
		input [31:0] adtre;
		input [31:0] sr29;
		input [31:0] sr31;
		input        cache_enable;
		begin
			case (regid)
				SYS_EIPC:  sysreg_read_fn = eipc;
				SYS_EIPSW: sysreg_read_fn = eipsw;
				SYS_FEPC:  sysreg_read_fn = fepc;
				SYS_FEPSW: sysreg_read_fn = fepsw;
				SYS_ECR:   sysreg_read_fn = ecr;
				SYS_PSW:   sysreg_read_fn = psw;
				SYS_PIR:   sysreg_read_fn = PIR_VALUE;
				SYS_TKCW:  sysreg_read_fn = TKCW_VALUE;
				SYS_CHCW:  sysreg_read_fn = chcw_readback_fn(cache_enable);
				SYS_ADTRE: sysreg_read_fn = adtre;
				SYS_SR29:  sysreg_read_fn = sr29;
				SYS_SR30:  sysreg_read_fn = SR30_VALUE;
				SYS_SR31:  sysreg_read_fn = sr31[31] ? (~sr31 + 32'd1) : sr31;
				default:   sysreg_read_fn = 32'd0;
			endcase
		end
	endfunction

	function [5:0] decode_uentry_fn;
		input [31:0] iw;
		begin
			// Unused instruction fields are ignored even when nonzero.
			if (iw[31:29] == OP_BCOND) begin
				decode_uentry_fn = UADDR_BCOND;
			end else begin
				case (iw[31:26])
					OP_MOV_R:  decode_uentry_fn = UADDR_MOV_R;
					OP_ADD_R:  decode_uentry_fn = UADDR_ADD_R;
					OP_SUB_R:  decode_uentry_fn = UADDR_SUB_R;
					OP_CMP_R:  decode_uentry_fn = UADDR_CMP_R;
					OP_SHL_R:  decode_uentry_fn = UADDR_SHL_R;
					OP_SHR_R:  decode_uentry_fn = UADDR_SHR_R;
					OP_JMP:    decode_uentry_fn = UADDR_JMP;
					OP_SAR_R:  decode_uentry_fn = UADDR_SAR_R;
					OP_MUL:    decode_uentry_fn = UADDR_MUL;
					OP_DIV:    decode_uentry_fn = UADDR_DIV;
					OP_MULU:   decode_uentry_fn = UADDR_MULU;
					OP_DIVU:   decode_uentry_fn = UADDR_DIVU;
					OP_OR_R:   decode_uentry_fn = UADDR_OR_R;
					OP_AND_R:  decode_uentry_fn = UADDR_AND_R;
					OP_XOR_R:  decode_uentry_fn = UADDR_XOR_R;
					OP_NOT_R:  decode_uentry_fn = UADDR_NOT_R;
					OP_MOV_I:  decode_uentry_fn = UADDR_MOV_I;
					OP_ADD_I:  decode_uentry_fn = UADDR_ADD_I;
					OP_SETF:   decode_uentry_fn = UADDR_SETF;
					OP_CMP_I:  decode_uentry_fn = UADDR_CMP_I;
					OP_SHL_I:  decode_uentry_fn = UADDR_SHL_I;
					OP_SHR_I:  decode_uentry_fn = UADDR_SHR_I;
					OP_CLI:    decode_uentry_fn = VB_EXT ? UADDR_CLI : UADDR_ILL;
					OP_SAR_I:  decode_uentry_fn = UADDR_SAR_I;
					OP_TRAP:   decode_uentry_fn = UADDR_TRAP;
					OP_RETI:   decode_uentry_fn = UADDR_RETI;
					OP_HALT:   decode_uentry_fn = UADDR_HALT;
					OP_LDSR:   decode_uentry_fn = UADDR_LDSR;
					OP_STSR:   decode_uentry_fn = UADDR_STSR;
					OP_SEI:    decode_uentry_fn = VB_EXT ? UADDR_SEI : UADDR_ILL;
					OP_BITSTR: begin
						// Bit-string subop is iw[20:16] in the packed fetch word.
						case (iw[20:16])
							SUBOP_SCH0BSU,
							SUBOP_SCH0BSD,
							SUBOP_SCH1BSU,
							SUBOP_SCH1BSD,
							SUBOP_ORBSU,
							SUBOP_ANDBSU,
							SUBOP_XORBSU,
							SUBOP_MOVBSU,
							SUBOP_ORNBSU,
							SUBOP_ANDNBSU,
							SUBOP_XORNBSU,
							SUBOP_NOTBSU: decode_uentry_fn = UADDR_BITSTR;
							default:      decode_uentry_fn = UADDR_ILL;
						endcase
					end
					OP_MOVEA:  decode_uentry_fn = UADDR_MOVEA;
					OP_ADDI:   decode_uentry_fn = UADDR_ADDI;
					OP_JR:     decode_uentry_fn = UADDR_JR;
					OP_JAL:    decode_uentry_fn = UADDR_JAL;
					OP_ORI:    decode_uentry_fn = UADDR_ORI;
					OP_ANDI:   decode_uentry_fn = UADDR_ANDI;
					OP_XORI:   decode_uentry_fn = UADDR_XORI;
					OP_MOVHI:  decode_uentry_fn = UADDR_MOVHI;
					OP_LD_B:   decode_uentry_fn = UADDR_MEM_LD_B;
					OP_LD_H:   decode_uentry_fn = UADDR_MEM_LD_H;
					OP_LD_W:   decode_uentry_fn = UADDR_MEM_LD_W;
					OP_ST_B:   decode_uentry_fn = UADDR_MEM_ST_B;
					OP_ST_H:   decode_uentry_fn = UADDR_MEM_ST_H;
					OP_ST_W:   decode_uentry_fn = UADDR_MEM_ST_W;
					OP_IN_B:   decode_uentry_fn = UADDR_MEM_IN_B;
					OP_IN_H:   decode_uentry_fn = UADDR_MEM_IN_H;
					OP_CAXI:   decode_uentry_fn = UADDR_CAXI;
					OP_IN_W:   decode_uentry_fn = UADDR_MEM_IN_W;
					OP_OUT_B:  decode_uentry_fn = UADDR_MEM_OUT_B;
					OP_OUT_H:  decode_uentry_fn = UADDR_MEM_OUT_H;
					OP_EXT: begin
						case (iw[15:10])
							SUBOP_CMPF:  decode_uentry_fn = UADDR_CMPF;
							SUBOP_CVTWS: decode_uentry_fn = UADDR_CVTWS;
							SUBOP_CVTSW,
							SUBOP_TRNCSW: decode_uentry_fn = UADDR_CVTSW;
							SUBOP_ADDF,
							SUBOP_SUBF,
							SUBOP_MULF,
							SUBOP_DIVF: decode_uentry_fn = UADDR_FPU;
							SUBOP_XB:    decode_uentry_fn = VB_EXT ? UADDR_XB : UADDR_ILL;
							SUBOP_XH:    decode_uentry_fn = VB_EXT ? UADDR_XH : UADDR_ILL;
							SUBOP_REV:   decode_uentry_fn = VB_EXT ? UADDR_REV : UADDR_ILL;
							SUBOP_MPYHW: decode_uentry_fn = VB_EXT ? UADDR_MPYHW : UADDR_ILL;
							default:     decode_uentry_fn = UADDR_ILL;
						endcase
					end
					OP_OUT_W:  decode_uentry_fn = UADDR_MEM_OUT_W;
					default:   decode_uentry_fn = UADDR_ILL;
				endcase
			end
		end
	endfunction

	function interrupt_accepted_fn;
		input       irq_valid;
		input [3:0] irq_level;
		input [31:0] psw;
		begin
			interrupt_accepted_fn =
				irq_valid &&
				((psw & PSW_ID_MASK) == 32'd0) &&
				((psw & PSW_EP_MASK) == 32'd0) &&
				((psw & PSW_NP_MASK) == 32'd0) &&
				(psw_interrupt_level_fn(psw) <= irq_level);
		end
	endfunction

	function nmi_accepted_fn;
		input        nmi_valid;
		input [31:0] psw;
		begin
			nmi_accepted_fn =
				nmi_valid &&
				((psw & PSW_NP_MASK) == 32'd0);
		end
	endfunction

	function [1:0] async_exception_kind_fn;
		input addr_trap;
		input nmi_accept;
		input irq_accept;
		begin
			if (nmi_accept) begin
				async_exception_kind_fn = ASYNC_EXC_NMI;
			end else if (irq_accept) begin
				async_exception_kind_fn = ASYNC_EXC_IRQ;
			end else if (addr_trap) begin
				async_exception_kind_fn = ASYNC_EXC_ADDR;
			end else begin
				async_exception_kind_fn = ASYNC_EXC_NONE;
			end
		end
	endfunction

	function [15:0] async_exception_code_fn;
		input [1:0] kind;
		input [3:0] irq_level;
		begin
			case (kind)
				ASYNC_EXC_ADDR: async_exception_code_fn = EXC_CODE_ADDRTRAP;
				ASYNC_EXC_NMI:  async_exception_code_fn = EXC_CODE_NMI;
				ASYNC_EXC_IRQ:  async_exception_code_fn = irq_code_fn(irq_level);
				default:        async_exception_code_fn = 16'd0;
			endcase
		end
	endfunction

	function [31:0] fatal_wdata_fn;
		input [1:0]  index;
		input [15:0] exc_code;
		input [31:0] psw;
		input [31:0] pc;
		begin
			case (index)
				2'd0: fatal_wdata_fn = {16'hffff, exc_code};
				2'd1: fatal_wdata_fn = psw;
				default: fatal_wdata_fn = pc;
			endcase
		end
	endfunction

	function [3:0] psw_interrupt_level_fn;
		input [31:0] psw;
		begin
			psw_interrupt_level_fn = psw[19:16];
		end
	endfunction

	function [15:0] irq_code_fn;
		input [3:0] irq_level;
		begin
			irq_code_fn = 16'hfe00 + {8'd0, irq_level, 4'b0000};
		end
	endfunction

	function [31:0] instr_len_bytes_fn;
		input [15:0] iw;
		begin
			instr_len_bytes_fn = instr_needs_second_half_fn(iw) ? 32'd4 : 32'd2;
		end
	endfunction

	function branch_taken_fn;
		input [3:0] cond;
		input [31:0] psw;
		reg zf;
		reg sf;
		reg ovf;
		reg cyf;
		begin
			zf = psw[0];
			sf = psw[1];
			ovf = psw[2];
			cyf = psw[3];
			case (cond)
				4'd0: branch_taken_fn = ovf;
				4'd1: branch_taken_fn = cyf;
				4'd2: branch_taken_fn = zf;
				4'd3: branch_taken_fn = cyf | zf;
				4'd4: branch_taken_fn = sf;
				4'd5: branch_taken_fn = 1'b1;
				4'd6: branch_taken_fn = ovf ^ sf;
				4'd7: branch_taken_fn = (ovf ^ sf) | zf;
				4'd8: branch_taken_fn = !ovf;
				4'd9: branch_taken_fn = !cyf;
				4'd10: branch_taken_fn = !zf;
				4'd11: branch_taken_fn = !(cyf | zf);
				4'd12: branch_taken_fn = !sf;
				4'd13: branch_taken_fn = 1'b0;
				4'd14: branch_taken_fn = !(ovf ^ sf);
				default: branch_taken_fn = !((ovf ^ sf) | zf);
			endcase
		end
	endfunction

	function [5:0] long_issue_cycles_fn;
		input [3:0] long_kind;
		begin
			case (long_kind)
				LONG_MUL,
				LONG_MULU:  long_issue_cycles_fn = 6'd13;
				LONG_DIV:   long_issue_cycles_fn = 6'd38;
				LONG_DIVU:  long_issue_cycles_fn = 6'd36;
				default:    long_issue_cycles_fn = 6'd0;
			endcase
		end
	endfunction

	// Bit-string word steps and datapath.
	function [5:0] bs_step_fn;
		input [31:0] len;
		input [4:0]  dst_ofs;
		reg [5:0] avail_v;
		begin
			avail_v = 6'd32 - {1'b0, dst_ofs};
			if (len == 32'd0) begin
				bs_step_fn = 6'd0;
			end else if (len < {26'd0, avail_v}) begin
				bs_step_fn = len[5:0];
			end else begin
				bs_step_fn = avail_v;
			end
		end
	endfunction

	function [5:0] bs_search_step_fn;
		input [31:0] len;
		input [4:0]  src_ofs;
		input        down;
		reg [5:0] avail_v;
		begin
			if (down) begin
				avail_v = {1'b0, src_ofs} + 6'd1;
			end else begin
				avail_v = 6'd32 - {1'b0, src_ofs};
			end
			if (len == 32'd0) begin
				bs_search_step_fn = 6'd0;
			end else if (len < {26'd0, avail_v}) begin
				bs_search_step_fn = len[5:0];
			end else begin
				bs_search_step_fn = avail_v;
			end
		end
	endfunction

	function [31:0] bs_search_addr_advance_fn;
		input [31:0] addr;
		input [4:0]  ofs;
		input [5:0]  bits;
		input        down;
		reg [6:0] sum_v;
		begin
			sum_v = 7'd0;
			if (down) begin
				if (bits > {1'b0, ofs}) begin
					bs_search_addr_advance_fn = addr - 32'd4;
				end else begin
					bs_search_addr_advance_fn = addr;
				end
			end else begin
				sum_v = {2'b00, ofs} + {1'b0, bits};
				bs_search_addr_advance_fn = addr + {29'd0, sum_v[5], 2'b00};
			end
		end
	endfunction

	function [4:0] bs_search_ofs_advance_fn;
		input [4:0] ofs;
		input [5:0] bits;
		input       down;
		reg [6:0] sum_v;
		begin
			sum_v = 7'd0;
			if (down) begin
				if (bits > {1'b0, ofs}) begin
					bs_search_ofs_advance_fn = 5'd31;
				end else begin
					bs_search_ofs_advance_fn = ofs - bits[4:0];
				end
			end else begin
				sum_v = {2'b00, ofs} + {1'b0, bits};
				bs_search_ofs_advance_fn = sum_v[4:0];
			end
		end
	endfunction

	function [31:0] bit_mask32_fn;
		input [5:0] bits;
		begin
			if (bits == 6'd0) begin
				bit_mask32_fn = 32'd0;
			end else if (bits >= 6'd32) begin
				bit_mask32_fn = 32'hffff_ffff;
			end else begin
				bit_mask32_fn = (32'h0000_0001 << bits) - 32'd1;
			end
		end
	endfunction

	function [31:0] bitstr_merge_aligned_fn;
		input [2:0]  op;
		input [31:0] src_aligned;
		input [31:0] dst_word;
		input [31:0] write_mask;
		reg [31:0] src_not_v;
		begin
			src_not_v = (~src_aligned) & write_mask;
			case (op)
				BS_OP_OR:   bitstr_merge_aligned_fn = dst_word | src_aligned;
				BS_OP_AND:  bitstr_merge_aligned_fn = dst_word & (~write_mask | src_aligned);
				BS_OP_XOR:  bitstr_merge_aligned_fn = dst_word ^ src_aligned;
				BS_OP_MOV:  bitstr_merge_aligned_fn = (dst_word & ~write_mask) | src_aligned;
				BS_OP_ORN:  bitstr_merge_aligned_fn = dst_word | src_not_v;
				BS_OP_ANDN: bitstr_merge_aligned_fn = dst_word & (~write_mask | src_not_v);
				BS_OP_XORN: bitstr_merge_aligned_fn = dst_word ^ src_not_v;
				default:    bitstr_merge_aligned_fn = (dst_word & ~write_mask) | src_not_v; // BS_OP_NOT
			endcase
		end
	endfunction

	function pipe_lag_activator_fn;
		input [5:0] op;
		begin
			case (op)
				OP_DIV, OP_DIVU, OP_MUL, OP_MULU, OP_SAR_R, OP_SAR_I, OP_SHL_R, OP_SHL_I,
				OP_SHR_R, OP_SHR_I, OP_SETF, OP_LDSR, OP_STSR, OP_EXT,
				OP_LD_B, OP_LD_H, OP_LD_W, OP_IN_B, OP_IN_H, OP_IN_W,
				OP_ST_B, OP_ST_H, OP_ST_W, OP_OUT_B, OP_OUT_H, OP_OUT_W:
					pipe_lag_activator_fn = 1'b1;
				default:
					pipe_lag_activator_fn = 1'b0;
			endcase
		end
	endfunction
