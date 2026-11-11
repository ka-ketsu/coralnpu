`ifndef HDL_VERILOG_RVV_DESIGN_RVV_SVH
`include "rvv_backend.svh"
`endif

module zvt_mt (
  clk,
  rst_n,
  readMtIdx,
  readSubIdx,
  readData,
`ifdef RVVI_ON
  rvviMtIdx,
  rvviSubIdx,
  rvviReadData,
`endif 
  writeEn, 
  writeMtIdx,
  writeSubIdx,
  writeData
);

// interface signals
  input  logic clk;
  input  logic rst_n;
  // read subtile     
  input  logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][$clog2(`NUM_MT)-1:0]      readMtIdx;
  input  logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][$clog2(`NUM_SUBTILE)-1:0] readSubIdx;
  output logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][`SUBTILE_SIZE*8-1:0]      readData;
`ifdef RVVI_ON
  // rvvi read subtile     
  input  logic [`NUM_BLK-1:0][3:0][$clog2(`NUM_MT)-1:0]                          rvviMtIdx;
  input  logic [`NUM_BLK-1:0][3:0][`NUM_SUBTILE/2-1:0][$clog2(`NUM_SUBTILE)-1:0] rvviSubIdx;
  output logic [`NUM_BLK-1:0][3:0][`NUM_SUBTILE/2-1:0][`SUBTILE_SIZE*8-1:0]      rvviReadData;
`endif 
  // write subtile
  input  logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][`SUBTILE_SIZE-1:0]        writeEn;  
  input  logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][$clog2(`NUM_MT)-1:0]      writeMtIdx;
  input  logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][$clog2(`NUM_SUBTILE)-1:0] writeSubIdx;
  input  logic [`NUM_BLK-1:0][`NUM_BLKPORT-1:0][`SUBTILE_SIZE*8-1:0]      writeData;    

// -------------- code start --------------------    
`ifdef FPGA_XILINX
  zvt_mt_fpga u_zvt_mt_fpga (.*);
`else
  // write data
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE-1:0]   wen;    // byte enable
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE*8-1:0] wdata;
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE*8-1:0] mt;

  // ---------------------------------------------------------------------------
  // 2-stage 256-bank sparse crossbar (VLEN=512 + ZVT_MAXCOMPUTING)
  //
  // The flat implementation gives every one of the 256 read ports a 4096:1 mux
  // and every one of the 4096 subtiles a 256-port priority mux (~284M 2:1 muxes
  // total). Under VLEN=512 + ZVT_MAXCOMPUTING, we group the 4096 subtiles into
  // 256 conflict-free logical banks of 16 rows using the bijection:
  //
  //   bank(m, s) = { m[1:0], s[5:2], s[1:0] ^ s[7:6] }
  //   row (m, s) = { m[3:2], s[7:6] }
  //
  // Inverse (bank k, row r) -> (m, s):
  //   m = { r[3:2], k[7:6] }
  //   s = { r[1:0], k[5:2], k[1:0] ^ r[1:0] }
  //
  // Across all PE, vtzero (EEW8/16/32), and MV/LSU (EEW8/16/32, pattern 0/1)
  // accesses in zvt_pe_block.sv and zvt_ctrl.sv:
  //  1. At most one active read port with row != 0 and at most one active write
  //     port ever targets a given bank k in any cycle (idle ports present m=0,
  //     s=0, whose row is 0, matching the default row 0).
  //  2. Each bank k only ever receives reads from ~14.4 candidate ports and
  //     writes from ~24.6 candidate ports; 240 of the 256 read ports (all
  //     non-LSU ports) only ever read from 4 PE banks (+ bank 0 when idle).
  //
  // Every array subscript below uses literal loop-constant integers so synthesis
  // never infers dynamic barrel-shifter ($shiftx) networks on the 524,288-bit
  // packed mt / wdata buses. Storage (zvt_mt_reg), reset behavior, RVVI reads,
  // and the module port list are completely unchanged.
  //
  // Other configurations, or `define ZVT_MT_FLAT, use the flat implementation.
  // ---------------------------------------------------------------------------
`ifdef VLEN_512
`ifdef ZVT_MAXCOMPUTING
`ifndef ZVT_MT_FLAT
`define ZVT_MT_USE_SPARSE
`endif
`endif
`endif

`ifdef ZVT_MT_USE_SPARSE
`undef ZVT_MT_USE_SPARSE
  localparam bit SPARSE = (`TE == 64) && (`NUM_MT == 16) && (`NUM_SUBTILE == 256) &&
                          (`SUBTILE_SIZE == 16) && (`NUM_BLK == 4) && (`NUM_BLKPORT == 64);
  localparam int NUM_BANK = 256;
  localparam int NUM_ROW  = 16;

  function automatic logic [7:0] get_bank(input logic [3:0] m, input logic [7:0] s);
    return {m[1:0], s[5:2], s[1:0] ^ s[7:6]};
  endfunction

  function automatic logic [3:0] get_row(input logic [3:0] m, input logic [7:0] s);
    return {m[3:2], s[7:6]};
  endfunction

  if (SPARSE) begin : g_sparse
    logic [NUM_BANK-1:0][3:0]                 bank_rrow;
    logic [NUM_BANK-1:0][`SUBTILE_SIZE*8-1:0] bank_rdata;
    logic [NUM_BANK-1:0][`SUBTILE_SIZE-1:0]   bank_wen;
    logic [NUM_BANK-1:0][3:0]                 bank_wrow;
    logic [NUM_BANK-1:0][`SUBTILE_SIZE*8-1:0] bank_wdata;

    // -------------------- Read Stage 1: per-bank row mux --------------------
    always_comb begin
      bank_rrow  = 'b0;
      bank_rdata = 'b0;

      for (int k = 0; k < NUM_BANK; k++) begin
        // Steer non-zero read row from the ~14.4 candidate ports of bank k.
        // Idle ports present (m=0, s=0) which has row 0 (the default).
        for (int b = 0; b < `NUM_BLK; b++) begin
          for (int p = 0; p < `NUM_BLKPORT; p++) begin
            if (// PE read candidate
                ((b == ((((k >> 7) & 1) << 1) | ((k >> 3) & 1))) &&
                 (((p >> 4) & 3) == ((k >> 4) & 3)) &&
                 (((p >> 3) & 1) == ((k >> 2) & 1)) &&
                 ((p & 1) == ((k >> 6) & 1))) ||
                // MV/LSU read candidate (block 0, ports 0..15)
                ((b == 0) && (p < 16) &&
                 ((((k >> 4) & 3) == (p & 3)) ||
                  (((k >> 2) & 3) == ((p >> 2) & 3)) ||
                  ((((k >> 6) & 1) == (p & 1)) && (((k >> 4) & 3) == ((p >> 1) & 3))) ||
                  ((p < 8) && ((((k >> 2) & 1) == ((p >> 2) & 1)) ||
                               (((k >> 6) & 1) == (p & 1))))))) begin
              if ((get_bank(readMtIdx[b][p], readSubIdx[b][p]) == 8'(k)) &&
                  (get_row(readMtIdx[b][p], readSubIdx[b][p]) != 4'd0)) begin
                bank_rrow[k] = get_row(readMtIdx[b][p], readSubIdx[b][p]);
              end
            end
          end
        end

        // 16:1 row mux for bank k (all mt subscripts are loop constants)
        for (int r = 0; r < NUM_ROW; r++) begin
          if (bank_rrow[k] == 4'(r)) begin
            bank_rdata[k] =
                mt[(((r >> 2) & 3) << 2) | ((k >> 6) & 3)]
                  [((r & 3) << 6) | (((k >> 2) & 15) << 2) | ((k & 3) ^ (r & 3))];
          end
        end
      end
    end

    // ----------------- Read Stage 2: bank-to-port demux ---------------------
    always_comb begin
      readData = 'b0;

      for (int b = 0; b < `NUM_BLK; b++) begin
        for (int p = 0; p < `NUM_BLKPORT; p++) begin
          // (m=0, s=0) <-> (bank=0, row=0) by bijection: route directly to
          // constant wire mt[0][0] so idle ports still read mt[0][0] even when
          // another port actively reads row != 0 of bank 0.
          readData[b][p] = mt[0][0];
          if ((readMtIdx[b][p] != '0) || (readSubIdx[b][p] != '0)) begin
            for (int k = 0; k < NUM_BANK; k++) begin
              if (// PE candidate banks (4 banks per port)
                  ((b == ((((k >> 7) & 1) << 1) | ((k >> 3) & 1))) &&
                   (((p >> 4) & 3) == ((k >> 4) & 3)) &&
                   (((p >> 3) & 1) == ((k >> 2) & 1)) &&
                   ((p & 1) == ((k >> 6) & 1))) ||
                  // MV/LSU candidate banks (block 0, ports 0..15 only)
                  ((b == 0) && (p < 16) &&
                   ((((k >> 4) & 3) == (p & 3)) ||
                    (((k >> 2) & 3) == ((p >> 2) & 3)) ||
                    ((((k >> 6) & 1) == (p & 1)) && (((k >> 4) & 3) == ((p >> 1) & 3))) ||
                    ((p < 8) && ((((k >> 2) & 1) == ((p >> 2) & 1)) ||
                                 (((k >> 6) & 1) == (p & 1))))))) begin
                if (get_bank(readMtIdx[b][p], readSubIdx[b][p]) == 8'(k)) begin
                  readData[b][p] = bank_rdata[k];
                end
              end
            end
          end
        end
      end
    end

    // ------------- Write Stage 1 & 2: per-bank steer + row decode -----------
    always_comb begin
      bank_wen   = 'b0;
      bank_wrow  = 'b0;
      bank_wdata = 'b0;
      wen        = 'b0;
      wdata      = 'b0;

      for (int k = 0; k < NUM_BANK; k++) begin
        // Steer from the ~24.6 candidate write ports of bank k in ascending
        // (b, p) order to match flat priority.
        for (int b = 0; b < `NUM_BLK; b++) begin
          for (int p = 0; p < `NUM_BLKPORT; p++) begin
            if (// PE write candidate
                ((b == ((((k >> 7) & 1) << 1) | ((k >> 3) & 1))) &&
                 (((p >> 4) & 3) == ((k >> 4) & 3)) &&
                 (((p >> 3) & 1) == ((k >> 2) & 1)) &&
                 ((p & 1) == ((k >> 6) & 1))) ||
                // zero EEW8 / EEW16 / EEW32 write candidates
                (p == ((k >> 2) & 15)) ||
                ((((b >> 1) & 1) == ((k >> 6) & 1)) &&
                 (((p >> 5) & 1) == 0) &&
                 (((p >> 1) & 15) == ((k >> 2) & 15))) ||
                ((b == ((k >> 6) & 3)) && (((p >> 2) & 15) == ((k >> 2) & 15))) ||
                // MV/LSU write candidate (block 0, ports 0..15)
                ((b == 0) && (p < 16) &&
                 ((((k >> 4) & 3) == (p & 3)) ||
                  (((k >> 2) & 3) == ((p >> 2) & 3)) ||
                  ((((k >> 6) & 1) == (p & 1)) && (((k >> 4) & 3) == ((p >> 1) & 3))) ||
                  ((p < 8) && ((((k >> 2) & 1) == ((p >> 2) & 1)) ||
                               (((k >> 6) & 1) == (p & 1))))))) begin
              if ((|writeEn[b][p]) &&
                  (get_bank(writeMtIdx[b][p], writeSubIdx[b][p]) == 8'(k))) begin
                bank_wen[k]   = writeEn[b][p];
                bank_wrow[k]  = get_row(writeMtIdx[b][p], writeSubIdx[b][p]);
                bank_wdata[k] = writeData[b][p];
              end
            end
          end
        end

        // Broadcast bank_wdata[k] (pure wire) and decode 4-bit row into wen
        for (int r = 0; r < NUM_ROW; r++) begin
          wdata[(((r >> 2) & 3) << 2) | ((k >> 6) & 3)]
               [((r & 3) << 6) | (((k >> 2) & 15) << 2) | ((k & 3) ^ (r & 3))] = bank_wdata[k];
          if (bank_wrow[k] == 4'(r)) begin
            wen[(((r >> 2) & 3) << 2) | ((k >> 6) & 3)]
               [((r & 3) << 6) | (((k >> 2) & 15) << 2) | ((k & 3) ^ (r & 3))] = bank_wen[k];
          end
        end
      end
    end

`ifdef ASSERT_ON
    // End-to-end invariant check: every port's target bank is in its candidate
    // set and has no row collision with another port in the same cycle.
    logic sparse_rd_ok;
    logic sparse_wr_ok;

    always_comb begin
      sparse_rd_ok = 1'b1;
      sparse_wr_ok = 1'b1;
      for (int b = 0; b < `NUM_BLK; b++) begin
        for (int p = 0; p < `NUM_BLKPORT; p++) begin
          logic [7:0] rk, wk;
          rk = get_bank(readMtIdx[b][p], readSubIdx[b][p]);
          wk = get_bank(writeMtIdx[b][p], writeSubIdx[b][p]);
          if ((readMtIdx[b][p] != '0) || (readSubIdx[b][p] != '0)) begin
            if (bank_rrow[rk] != get_row(readMtIdx[b][p], readSubIdx[b][p])) begin
              sparse_rd_ok = 1'b0;
            end
            if (!(((b == {rk[7], rk[3]}) && (p[5:4] == rk[5:4]) &&
                   (p[3] == rk[2]) && (p[0] == rk[6])) ||
                  ((b == 0) && (p < 16) &&
                   ((rk[5:4] == p[1:0]) || (rk[3:2] == p[3:2]) ||
                    ((rk[6] == p[0]) && (rk[5:4] == p[2:1])) ||
                    (!p[3] && ((rk[2] == p[2]) || (rk[6] == p[0]))))))) begin
              sparse_rd_ok = 1'b0;
            end
          end
          if (|writeEn[b][p]) begin
            if (bank_wrow[wk] != get_row(writeMtIdx[b][p], writeSubIdx[b][p])) begin
              sparse_wr_ok = 1'b0;
            end
            if (!(((b == {wk[7], wk[3]}) && (p[5:4] == wk[5:4]) &&
                   (p[3] == wk[2]) && (p[0] == wk[6])) ||
                  (p == {2'b00, wk[5:2]}) ||
                  ((b[1] == wk[6]) && !p[5] && (p[4:1] == wk[5:2])) ||
                  ((b == wk[7:6]) && (p[5:2] == wk[5:2])) ||
                  ((b == 0) && (p < 16) &&
                   ((wk[5:4] == p[1:0]) || (wk[3:2] == p[3:2]) ||
                    ((wk[6] == p[0]) && (wk[5:4] == p[2:1])) ||
                    (!p[3] && ((wk[2] == p[2]) || (wk[6] == p[0]))))))) begin
              sparse_wr_ok = 1'b0;
            end
          end
        end
      end
    end

    property p_sparse_rd_ok;
      @(posedge clk) disable iff (!rst_n) sparse_rd_ok;
    endproperty
    assert property (p_sparse_rd_ok)
      else $error("ZVT_MT: read port address outside sparse bank map or bank row collision!");

    property p_sparse_wr_ok;
      @(posedge clk) disable iff (!rst_n) sparse_wr_ok;
    endproperty
    assert property (p_sparse_wr_ok)
      else $error("ZVT_MT: write port address outside sparse bank map or bank row collision!");
`endif

  end
`else
  localparam bit SPARSE = 1'b0;

  always_comb begin
    wen      = 'b0;
    wdata    = 'b0;
    readData = 'b0;

    for(int i=0; i<`NUM_BLK; i++) begin: block_addr
      for(int j=0; j<`NUM_BLKPORT; j++) begin: block_subtile_addr
        // write
        // Multiple (i,j) entries can map to the same (mt, sub) slot, and the
        // unused entries default to writeMtIdx=0, writeSubIdx=0, writeEn=0.
        // A blind "last writer wins" assignment lets those zero entries stomp
        // the valid write from a lower (i,j). Guard on writeEn so only entries
        // actually requesting a write contribute.
        if (|writeEn[i][j]) begin
          wen[  writeMtIdx[i][j]][writeSubIdx[i][j]] = writeEn[i][j];
          wdata[writeMtIdx[i][j]][writeSubIdx[i][j]] = writeData[i][j];
        end
        // read
        readData[i][j] = mt[readMtIdx[i][j]][readSubIdx[i][j]];
      end
    end
  end
`endif

`ifdef RVVI_ON
  // rvvi read subtile     
  always_comb begin
    for(int i=0;i<`NUM_BLK;i++) begin
      for(int j=0;j<4;j++) begin
        for(int k=0;k<`NUM_SUBTILE/2;k++) begin
          rvviReadData[i][j][k] = mt[rvviMtIdx[i][j]][rvviSubIdx[i][j][k]];
        end
      end
    end
  end
`endif 

  zvt_mt_reg mtReg (
    .clk    (clk),
    .rst_n  (rst_n),
    .wen    (wen),
    .wdata  (wdata),
    .mt     (mt)
  );
`endif

endmodule
