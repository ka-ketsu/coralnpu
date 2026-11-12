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
  // ---------------------------------------------------------------------------
  // 2-stage 256-bank sparse crossbar + 16 x 4-KB tile hierarchy
  // (VLEN=512 + ZVT_MAXCOMPUTING)
  //
  // Partitions the 4,096 subtiles (64 KB) into 256 conflict-free logical banks
  // of 16 rows grouped into 16 x zvt_mt_tile (4 KB each), with 16 x
  // zvt_mt_subtile (256 B each) per tile:
  //
  //   bank(m, s) = { m[1:0], s[5:2], s[1:0] ^ s[7:6] }   (tile = bank[7:4],
  //   row (m, s) = { m[3:2], s[7:6] }                     sub  = bank[3:0])
  //
  // Inverse (bank k, row r) -> (m, s):
  //   m = { r[3:2], k[7:6] }
  //   s = { r[1:0], k[5:2], k[1:0] ^ r[1:0] }
  //
  // Each zvt_mt_subtile encapsulates its 16x16B edff storage, local 4-to-16
  // write-row decoder, and local 16-to-1 128b read-row mux for bottom-up
  // synthesis. Other configurations (or `define ZVT_MT_FLAT) fall back to the
  // legacy flat zvt_mt_reg implementation.
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
  localparam int unsigned NUM_BANK      = 32'd256;
  localparam int unsigned NUM_BLK_U     = 32'd4;
  localparam int unsigned NUM_BLKPORT_U = 32'd64;

  function automatic logic [7:0] get_bank(input logic [1:0] m_lo, input logic [7:0] s);
    get_bank = {m_lo, s[5:2], s[1:0] ^ s[7:6]};
  endfunction

  function automatic logic [3:0] get_row(input logic [1:0] m_hi, input logic [1:0] s_hi);
    get_row = {m_hi, s_hi};
  endfunction

  logic [255:0][3:0]                 bank_rrow;
  logic [255:0][`SUBTILE_SIZE*8-1:0] bank_rdata;
  logic [255:0][`SUBTILE_SIZE-1:0]   bank_wen;
  logic [255:0][3:0]                 bank_wrow;
  logic [255:0][`SUBTILE_SIZE*8-1:0] bank_wdata;
  /* verilator lint_off UNUSEDSIGNAL */
  logic [15:0][`SUBTILE_SIZE*8-1:0]  tile_row0_rdata;
  /* verilator lint_on UNUSEDSIGNAL */
`ifdef RVVI_ON
  logic [15:0][15:0][15:0][`SUBTILE_SIZE*8-1:0] rvvi_all_rows;
`endif

  // ---------------- Read Stage 1: port-to-bank row address steer -------------
  always_comb begin
    bank_rrow = '0;
    for (int unsigned k = 32'd0; k < NUM_BANK; k = k + 32'd1) begin
      for (int unsigned b = 32'd0; b < NUM_BLK_U; b = b + 32'd1) begin
        for (int unsigned p = 32'd0; p < NUM_BLKPORT_U; p = p + 32'd1) begin
          if (// PE read candidate
              ((b[1:0] == {k[7], k[3]}) &&
               (p[5:4] == k[5:4]) &&
               (p[3]   == k[2]) &&
               (p[0]   == k[6])) ||
              // MV/LSU read candidate (block 0, ports 0..15)
              ((b == 32'd0) && (p < 32'd16) &&
               ((k[5:4] == p[1:0]) ||
                (k[3:2] == p[3:2]) ||
                ((k[6] == p[0]) && (k[5:4] == p[2:1])) ||
                (!p[3] && ((k[2] == p[2]) || (k[6] == p[0])))))) begin
            if ((get_bank(readMtIdx[b][p][1:0], readSubIdx[b][p]) == k[7:0]) &&
                (get_row(readMtIdx[b][p][3:2], readSubIdx[b][p][7:6]) != 4'd0)) begin
              bank_rrow[k] = get_row(readMtIdx[b][p][3:2], readSubIdx[b][p][7:6]);
            end
          end
        end
      end
    end
  end

  // ----------------- Read Stage 2: bank-to-port data demux -------------------
  always_comb begin
    readData = '0;
    for (int unsigned b = 32'd0; b < NUM_BLK_U; b = b + 32'd1) begin
      for (int unsigned p = 32'd0; p < NUM_BLKPORT_U; p = p + 32'd1) begin
        // (m=0, s=0) <-> (bank=0, row=0) by bijection: tap tile 0 / subtile 0 /
        // row 0 directly so idle ports read mt[0][0] even when another port
        // actively reads row != 0 of bank 0.
        readData[b][p] = tile_row0_rdata[0];
        if ((readMtIdx[b][p] != 4'd0) || (readSubIdx[b][p] != 8'd0)) begin
          for (int unsigned k = 32'd0; k < NUM_BANK; k = k + 32'd1) begin
            if (// PE candidate banks (4 banks per port)
                ((b[1:0] == {k[7], k[3]}) &&
                 (p[5:4] == k[5:4]) &&
                 (p[3]   == k[2]) &&
                 (p[0]   == k[6])) ||
                // MV/LSU candidate banks (block 0, ports 0..15)
                ((b == 32'd0) && (p < 32'd16) &&
                 ((k[5:4] == p[1:0]) ||
                  (k[3:2] == p[3:2]) ||
                  ((k[6] == p[0]) && (k[5:4] == p[2:1])) ||
                  (!p[3] && ((k[2] == p[2]) || (k[6] == p[0])))))) begin
              if (get_bank(readMtIdx[b][p][1:0], readSubIdx[b][p]) == k[7:0]) begin
                readData[b][p] = bank_rdata[k];
              end
            end
          end
        end
      end
    end
  end

  // ----------------- Write Stage 1: port-to-bank write steer -----------------
  always_comb begin
    bank_wen   = '0;
    bank_wrow  = '0;
    bank_wdata = '0;
    for (int unsigned k = 32'd0; k < NUM_BANK; k = k + 32'd1) begin
      for (int unsigned b = 32'd0; b < NUM_BLK_U; b = b + 32'd1) begin
        for (int unsigned p = 32'd0; p < NUM_BLKPORT_U; p = p + 32'd1) begin
          if (// PE write candidate
              ((b[1:0] == {k[7], k[3]}) &&
               (p[5:4] == k[5:4]) &&
               (p[3]   == k[2]) &&
               (p[0]   == k[6])) ||
              // zero EEW8 / EEW16 / EEW32 write candidates
              (p[5:0] == {2'b00, k[5:2]}) ||
              ((b[1] == k[6]) && !p[5] && (p[4:1] == k[5:2])) ||
              ((b[1:0] == k[7:6]) && (p[5:2] == k[5:2])) ||
              // MV/LSU write candidate (block 0, ports 0..15)
              ((b == 32'd0) && (p < 32'd16) &&
               ((k[5:4] == p[1:0]) ||
                (k[3:2] == p[3:2]) ||
                ((k[6] == p[0]) && (k[5:4] == p[2:1])) ||
                (!p[3] && ((k[2] == p[2]) || (k[6] == p[0])))))) begin
            if ((|writeEn[b][p]) &&
                (get_bank(writeMtIdx[b][p][1:0], writeSubIdx[b][p]) == k[7:0])) begin
              bank_wen[k]   = writeEn[b][p];
              bank_wrow[k]  = get_row(writeMtIdx[b][p][3:2], writeSubIdx[b][p][7:6]);
              bank_wdata[k] = writeData[b][p];
            end
          end
        end
      end
    end
  end

  // --------------- 16 x 4-KB hierarchical tile instantiations ----------------
  genvar t_g;
  generate
    for (t_g = 0; t_g < 16; t_g++) begin : gen_tile
      zvt_mt_tile u_tile (
        .clk            (clk),
        .rst_n          (rst_n),
        .rrow           (bank_rrow[t_g*16 +: 16]),
        .rdata          (bank_rdata[t_g*16 +: 16]),
        .row0_rdata     (tile_row0_rdata[t_g]),
      `ifdef RVVI_ON
        .rvvi_tile_rows (rvvi_all_rows[t_g]),
      `endif
        .wen            (bank_wen[t_g*16 +: 16]),
        .wrow           (bank_wrow[t_g*16 +: 16]),
        .wdata          (bank_wdata[t_g*16 +: 16])
      );
    end
  endgenerate

`ifdef RVVI_ON
  localparam int unsigned NUM_TILE         = 32'd16;
  localparam int unsigned NUM_SUB_PER_TILE = 32'd16;
  localparam int unsigned NUM_ROW          = 32'd16;
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE*8-1:0] mt_rvvi;

  always_comb begin
    mt_rvvi = '0;
    for (int unsigned t_i = 32'd0; t_i < NUM_TILE; t_i = t_i + 32'd1) begin
      for (int unsigned u_i = 32'd0; u_i < NUM_SUB_PER_TILE; u_i = u_i + 32'd1) begin
        for (int unsigned r_i = 32'd0; r_i < NUM_ROW; r_i = r_i + 32'd1) begin
          mt_rvvi[{r_i[3:2], t_i[3:2]}]
                 [{r_i[1:0], t_i[1:0], u_i[3:2], u_i[1:0] ^ r_i[1:0]}] =
              rvvi_all_rows[t_i][u_i][r_i];
        end
      end
    end
    for (int unsigned i = 32'd0; i < NUM_BLK_U; i = i + 32'd1) begin
      for (int unsigned j = 32'd0; j < 32'd4; j = j + 32'd1) begin
        for (int unsigned k = 32'd0; k < 32'd128; k = k + 32'd1) begin
          rvviReadData[i][j][k] = mt_rvvi[rvviMtIdx[i][j]][rvviSubIdx[i][j][k]];
        end
      end
    end
  end
`endif

`ifdef ASSERT_ON
  // End-to-end invariant check: every port's target bank is in its candidate
  // set and has no row collision with another port in the same cycle.
  logic sparse_rd_ok;
  logic sparse_wr_ok;

  always_comb begin
    sparse_rd_ok = 1'b1;
    sparse_wr_ok = 1'b1;
    for (int unsigned b = 32'd0; b < NUM_BLK_U; b = b + 32'd1) begin
      for (int unsigned p = 32'd0; p < NUM_BLKPORT_U; p = p + 32'd1) begin
        logic [7:0] rk, wk;
        rk = get_bank(readMtIdx[b][p][1:0], readSubIdx[b][p]);
        wk = get_bank(writeMtIdx[b][p][1:0], writeSubIdx[b][p]);
        if ((readMtIdx[b][p] != 4'd0) || (readSubIdx[b][p] != 8'd0)) begin
          if (bank_rrow[rk] != get_row(readMtIdx[b][p][3:2], readSubIdx[b][p][7:6])) begin
            sparse_rd_ok = 1'b0;
          end
          if (!(((b[1:0] == {rk[7], rk[3]}) && (p[5:4] == rk[5:4]) &&
                 (p[3] == rk[2]) && (p[0] == rk[6])) ||
                ((b == 32'd0) && (p < 32'd16) &&
                 ((rk[5:4] == p[1:0]) || (rk[3:2] == p[3:2]) ||
                  ((rk[6] == p[0]) && (rk[5:4] == p[2:1])) ||
                  (!p[3] && ((rk[2] == p[2]) || (rk[6] == p[0]))))))) begin
            sparse_rd_ok = 1'b0;
          end
        end
        if (|writeEn[b][p]) begin
          if (bank_wrow[wk] != get_row(writeMtIdx[b][p][3:2], writeSubIdx[b][p][7:6])) begin
            sparse_wr_ok = 1'b0;
          end
          if (!(((b[1:0] == {wk[7], wk[3]}) && (p[5:4] == wk[5:4]) &&
                 (p[3] == wk[2]) && (p[0] == wk[6])) ||
                (p[5:0] == {2'b00, wk[5:2]}) ||
                ((b[1] == wk[6]) && !p[5] && (p[4:1] == wk[5:2])) ||
                ((b[1:0] == wk[7:6]) && (p[5:2] == wk[5:2])) ||
                ((b == 32'd0) && (p < 32'd16) &&
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

`else
  // Legacy flat implementation for non-(VLEN=512 + ZVT_MAXCOMPUTING) or ZVT_MT_FLAT
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE-1:0]   wen;    // byte enable
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE*8-1:0] wdata;
  logic [`NUM_MT-1:0][`NUM_SUBTILE-1:0][`SUBTILE_SIZE*8-1:0] mt;

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
`endif

endmodule
