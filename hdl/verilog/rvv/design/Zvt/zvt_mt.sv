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
  localparam int unsigned NUM_BANK = 32'd256;

  function automatic logic [7:0] get_bank(input logic [1:0] m_lo, input logic [7:0] s);
    get_bank = {m_lo, s[5:2], s[1:0] ^ s[7:6]};
  endfunction

  function automatic logic [3:0] get_row(input logic [1:0] m_hi, input logic [1:0] s_hi);
    get_row = {m_hi, s_hi};
  endfunction

  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic is_pe_port(
    input logic [1:0] b,
    input logic [5:0] p,
    input logic [7:0] k
  );
    is_pe_port = (b == {k[7], k[3]}) &&
                 (p[5:4] == k[5:4]) &&
                 (p[3]   == k[2]) &&
                 (p[0]   == k[6]);
  endfunction

  function automatic logic is_mvlsu_port(
    input logic [1:0] b,
    input logic [5:0] p,
    input logic [7:0] k
  );
    is_mvlsu_port = (b == 2'd0) && (p[5:4] == 2'd0) &&
                    ((k[5:4] == p[1:0]) ||
                     (k[3:2] == p[3:2]) ||
                     ((k[6] == p[0]) && (k[5:4] == p[2:1])) ||
                     (!p[3] && ((k[2] == p[2]) || (k[6] == p[0]))));
  endfunction

  function automatic logic is_z8_port(
    input logic [5:0] p,
    input logic [7:0] k
  );
    is_z8_port = (p == {2'b00, k[5:2]});
  endfunction

  function automatic logic is_z16_port(
    input logic [1:0] b,
    input logic [5:0] p,
    input logic [7:0] k
  );
    is_z16_port = (b[1] == k[6]) && !p[5] && (p[4:1] == k[5:2]);
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  logic [3:0][63:0][7:0]             rd_bank;
  logic [3:0][63:0][3:0]             rd_row;
  logic [3:0][63:0]                  rd_nz;
  logic [3:0][63:0][7:0]             wr_bank;
  logic [3:0][63:0][3:0]             wr_row;
  logic [3:0][63:0]                  wr_vld;

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

  // ------ Per-port address decode + Read Stage 2 (bank-to-port demux) --------
  genvar gb, gp;
  generate
    for (gb = 0; gb < 4; gb++) begin : gen_port_b
      localparam logic [1:0] B2 = (((gb & 2) != 0) ? 2'h2 : 2'h0) |
                                  (((gb & 1) != 0) ? 2'h1 : 2'h0);
      for (gp = 0; gp < 64; gp++) begin : gen_port_p
        localparam logic [5:0] P6 = (((gp & 32) != 0) ? 6'h20 : 6'h00) |
                                    (((gp & 16) != 0) ? 6'h10 : 6'h00) |
                                    (((gp &  8) != 0) ? 6'h08 : 6'h00) |
                                    (((gp &  4) != 0) ? 6'h04 : 6'h00) |
                                    (((gp &  2) != 0) ? 6'h02 : 6'h00) |
                                    (((gp &  1) != 0) ? 6'h01 : 6'h00);

        assign rd_bank[gb][gp] = get_bank(readMtIdx[gb][gp][1:0], readSubIdx[gb][gp]);
        assign rd_row[gb][gp]  = get_row(readMtIdx[gb][gp][3:2], readSubIdx[gb][gp][7:6]);
        assign rd_nz[gb][gp]   = (readMtIdx[gb][gp] != 4'd0) || (readSubIdx[gb][gp] != 8'd0);
        assign wr_bank[gb][gp] = get_bank(writeMtIdx[gb][gp][1:0], writeSubIdx[gb][gp]);
        assign wr_row[gb][gp]  = get_row(writeMtIdx[gb][gp][3:2], writeSubIdx[gb][gp][7:6]);
        assign wr_vld[gb][gp]  = |writeEn[gb][gp];

        // (m=0, s=0) <-> (bank=0, row=0) by bijection: tap tile 0 / subtile 0 /
        // row 0 directly so idle ports read mt[0][0] even when another port
        // actively reads row != 0 of bank 0.
        if ((gb == 0) && (gp < 16)) begin : gen_rd_mvlsu
          logic [`SUBTILE_SIZE*8-1:0] port_rdata;

          always_comb begin
            logic [`SUBTILE_SIZE*8-1:0] rdata_acc;
            logic                       rsel;
            rdata_acc = '0;
            rsel      = 1'b0;
            for (int unsigned kk = 32'd0; kk < NUM_BANK; kk = kk + 32'd1) begin
              if (is_pe_port(2'd0, P6, kk[7:0]) || is_mvlsu_port(2'd0, P6, kk[7:0])) begin
                rsel      = (rd_bank[0][gp] == kk[7:0]);
                rdata_acc = rdata_acc | ({(`SUBTILE_SIZE*8){rsel}} & bank_rdata[kk]);
              end
            end
            port_rdata = rd_nz[0][gp] ? rdata_acc : tile_row0_rdata[0];
          end

          assign readData[0][gp] = port_rdata;
        end else begin : gen_rd_pe
          localparam int K_BASE = ((gb / 2) * 128) + ((gp % 2) * 64) +
                                  ((gp / 16) * 16) + ((gb % 2) * 8) +
                                  (((gp / 8) % 2) * 4);
          localparam logic [5:0] K_HI = {B2[1], P6[0], P6[5:4], B2[0], P6[3]};

          assign readData[gb][gp] =
              !(rd_nz[gb][gp] && (rd_bank[gb][gp][7:2] == K_HI)) ? tile_row0_rdata[0] :
              (rd_bank[gb][gp][1:0] == 2'd0)                     ? bank_rdata[K_BASE + 0] :
              (rd_bank[gb][gp][1:0] == 2'd1)                     ? bank_rdata[K_BASE + 1] :
              (rd_bank[gb][gp][1:0] == 2'd2)                     ? bank_rdata[K_BASE + 2] :
                                                                   bank_rdata[K_BASE + 3];
        end
      end
    end
  endgenerate

  // ------ Per-bank Read Stage 1 & Write Stage 1 (port-to-bank steer) ---------
  genvar gk;
  generate
    for (gk = 0; gk < 256; gk++) begin : gen_bank
      localparam logic [7:0] K = (((gk & 128) != 0) ? 8'h80 : 8'h00) |
                                 (((gk &  64) != 0) ? 8'h40 : 8'h00) |
                                 (((gk &  32) != 0) ? 8'h20 : 8'h00) |
                                 (((gk &  16) != 0) ? 8'h10 : 8'h00) |
                                 (((gk &   8) != 0) ? 8'h08 : 8'h00) |
                                 (((gk &   4) != 0) ? 8'h04 : 8'h00) |
                                 (((gk &   2) != 0) ? 8'h02 : 8'h00) |
                                 (((gk &   1) != 0) ? 8'h01 : 8'h00);

      logic [3:0]                 k_rrow;
      logic [`SUBTILE_SIZE-1:0]   k_wen;
      logic [3:0]                 k_wrow;
      logic [`SUBTILE_SIZE*8-1:0] k_wdata;

      // Read Stage 1: flat AND-OR reduction over <= 17 candidate ports
      always_comb begin
        logic [3:0] rrow_acc;
        logic [1:0] rb;
        logic [5:0] rp;
        logic       rsel;
        rrow_acc = 4'd0;
        rb       = 2'd0;
        rp       = 6'd0;
        rsel     = 1'b0;

        // Group 1: PE read candidates (4 ports)
        rb = {K[7], K[3]};
        for (int unsigned idx = 32'd0; idx < 32'd4; idx = idx + 32'd1) begin
          rp       = {K[5:4], K[2], idx[1:0], K[6]};
          rsel     = (rd_bank[rb][rp] == K);
          rrow_acc = rrow_acc | ({4{rsel}} & rd_row[rb][rp]);
        end

        // Group 2: MV/LSU read candidates (block 0, ports 0..15)
        for (int unsigned pp = 32'd0; pp < 32'd16; pp = pp + 32'd1) begin
          rp = {2'b00, pp[3:0]};
          if (is_mvlsu_port(2'd0, rp, K) && !is_pe_port(2'd0, rp, K)) begin
            rsel     = (rd_bank[2'd0][rp] == K);
            rrow_acc = rrow_acc | ({4{rsel}} & rd_row[2'd0][rp]);
          end
        end

        k_rrow = rrow_acc;
      end

      // Write Stage 1: flat AND-OR reduction over <= 28 candidate ports
      always_comb begin
        logic [`SUBTILE_SIZE-1:0]   wen_acc;
        logic [3:0]                 wrow_acc;
        logic [`SUBTILE_SIZE*8-1:0] wdata_acc;
        logic [1:0]                 wb;
        logic [5:0]                 wp;
        logic                       wsel;
        wen_acc   = '0;
        wrow_acc  = '0;
        wdata_acc = '0;
        wb        = 2'd0;
        wp        = 6'd0;
        wsel      = 1'b0;

        // Group 1: PE write candidates (4 ports)
        wb = {K[7], K[3]};
        for (int unsigned idx = 32'd0; idx < 32'd4; idx = idx + 32'd1) begin
          wp        = {K[5:4], K[2], idx[1:0], K[6]};
          wsel      = wr_vld[wb][wp] && (wr_bank[wb][wp] == K);
          wen_acc   = wen_acc   | ({`SUBTILE_SIZE{wsel}}     & writeEn[wb][wp]);
          wrow_acc  = wrow_acc  | ({4{wsel}}                 & wr_row[wb][wp]);
          wdata_acc = wdata_acc | ({(`SUBTILE_SIZE*8){wsel}} & writeData[wb][wp]);
        end

        // Group 2: MV/LSU write candidates (block 0, ports 0..15)
        for (int unsigned pp = 32'd0; pp < 32'd16; pp = pp + 32'd1) begin
          wp = {2'b00, pp[3:0]};
          if (is_mvlsu_port(2'd0, wp, K) && !is_pe_port(2'd0, wp, K)) begin
            wsel      = wr_vld[2'd0][wp] && (wr_bank[2'd0][wp] == K);
            wen_acc   = wen_acc   | ({`SUBTILE_SIZE{wsel}}     & writeEn[2'd0][wp]);
            wrow_acc  = wrow_acc  | ({4{wsel}}                 & wr_row[2'd0][wp]);
            wdata_acc = wdata_acc | ({(`SUBTILE_SIZE*8){wsel}} & writeData[2'd0][wp]);
          end
        end

        // Group 3: vtzero EEW8 write candidates (4 ports)
        wp = {2'b00, K[5:2]};
        for (int unsigned zb = 32'd0; zb < 32'd4; zb = zb + 32'd1) begin
          wb = zb[1:0];
          if (!is_pe_port(wb, wp, K) && !is_mvlsu_port(wb, wp, K)) begin
            wsel      = wr_vld[wb][wp] && (wr_bank[wb][wp] == K);
            wen_acc   = wen_acc   | ({`SUBTILE_SIZE{wsel}}     & writeEn[wb][wp]);
            wrow_acc  = wrow_acc  | ({4{wsel}}                 & wr_row[wb][wp]);
            wdata_acc = wdata_acc | ({(`SUBTILE_SIZE*8){wsel}} & writeData[wb][wp]);
          end
        end

        // Group 4: vtzero EEW16 write candidates (4 ports)
        for (int unsigned zb = 32'd0; zb < 32'd2; zb = zb + 32'd1) begin
          for (int unsigned zp = 32'd0; zp < 32'd2; zp = zp + 32'd1) begin
            wb = {K[6], zb[0]};
            wp = {1'b0, K[5:2], zp[0]};
            if (!is_pe_port(wb, wp, K) &&
                !is_mvlsu_port(wb, wp, K) &&
                !is_z8_port(wp, K)) begin
              wsel      = wr_vld[wb][wp] && (wr_bank[wb][wp] == K);
              wen_acc   = wen_acc   | ({`SUBTILE_SIZE{wsel}}     & writeEn[wb][wp]);
              wrow_acc  = wrow_acc  | ({4{wsel}}                 & wr_row[wb][wp]);
              wdata_acc = wdata_acc | ({(`SUBTILE_SIZE*8){wsel}} & writeData[wb][wp]);
            end
          end
        end

        // Group 5: vtzero EEW32 write candidates (4 ports)
        wb = K[7:6];
        for (int unsigned zp = 32'd0; zp < 32'd4; zp = zp + 32'd1) begin
          wp = {K[5:2], zp[1:0]};
          if (!is_pe_port(wb, wp, K) &&
              !is_mvlsu_port(wb, wp, K) &&
              !is_z8_port(wp, K) &&
              !is_z16_port(wb, wp, K)) begin
            wsel      = wr_vld[wb][wp] && (wr_bank[wb][wp] == K);
            wen_acc   = wen_acc   | ({`SUBTILE_SIZE{wsel}}     & writeEn[wb][wp]);
            wrow_acc  = wrow_acc  | ({4{wsel}}                 & wr_row[wb][wp]);
            wdata_acc = wdata_acc | ({(`SUBTILE_SIZE*8){wsel}} & writeData[wb][wp]);
          end
        end

        k_wen   = wen_acc;
        k_wrow  = wrow_acc;
        k_wdata = wdata_acc;
      end

      assign bank_rrow[gk]  = k_rrow;
      assign bank_wen[gk]   = k_wen;
      assign bank_wrow[gk]  = k_wrow;
      assign bank_wdata[gk] = k_wdata;
    end
  endgenerate

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
    for (int unsigned i = 32'd0; i < 32'd4; i = i + 32'd1) begin
      for (int unsigned j = 32'd0; j < 32'd4; j = j + 32'd1) begin
        for (int unsigned k = 32'd0; k < 32'd128; k = k + 32'd1) begin
          rvviReadData[i][j][k] = mt_rvvi[rvviMtIdx[i][j]][rvviSubIdx[i][j][k]];
        end
      end
    end
  end
`endif

`ifdef ASSERT_ON
  // End-to-end invariant check: every active port's target bank is in its
  // candidate set and at most 1 active read / 1 active write port hits a bank.
  logic sparse_rd_ok;
  logic sparse_wr_ok;

  always_comb begin
    logic [255:0][8:0] rd_bank_cnt;
    logic [255:0][8:0] wr_bank_cnt;
    sparse_rd_ok = 1'b1;
    sparse_wr_ok = 1'b1;
    rd_bank_cnt  = '0;
    wr_bank_cnt  = '0;
    for (int unsigned b = 32'd0; b < 32'd4; b = b + 32'd1) begin
      for (int unsigned p = 32'd0; p < 32'd64; p = p + 32'd1) begin
        logic [7:0] rk, wk;
        rk = rd_bank[b][p];
        wk = wr_bank[b][p];
        if (rd_nz[b][p]) begin
          rd_bank_cnt[rk] = rd_bank_cnt[rk] + 9'd1;
          if (bank_rrow[rk] != rd_row[b][p]) begin
            sparse_rd_ok = 1'b0;
          end
          if (!(is_pe_port(b[1:0], p[5:0], rk) ||
                is_mvlsu_port(b[1:0], p[5:0], rk))) begin
            sparse_rd_ok = 1'b0;
          end
        end
        if (wr_vld[b][p]) begin
          wr_bank_cnt[wk] = wr_bank_cnt[wk] + 9'd1;
          if (bank_wrow[wk] != wr_row[b][p]) begin
            sparse_wr_ok = 1'b0;
          end
          if (!(is_pe_port(b[1:0], p[5:0], wk) ||
                is_mvlsu_port(b[1:0], p[5:0], wk) ||
                is_z8_port(p[5:0], wk) ||
                is_z16_port(b[1:0], p[5:0], wk) ||
                ((b[1:0] == wk[7:6]) && (p[5:2] == wk[5:2])))) begin
            sparse_wr_ok = 1'b0;
          end
        end
      end
    end
    for (int unsigned k = 32'd0; k < NUM_BANK; k = k + 32'd1) begin
      if (rd_bank_cnt[k] > 9'd1) begin
        sparse_rd_ok = 1'b0;
      end
      if (wr_bank_cnt[k] > 9'd1) begin
        sparse_wr_ok = 1'b0;
      end
    end
  end

  /* verilator lint_off SYNCASYNCNET */
  property p_sparse_rd_ok;
    @(posedge clk) disable iff (!rst_n) sparse_rd_ok;
  endproperty
  assert property (p_sparse_rd_ok)
    else $error("ZVT_MT: read port address outside sparse bank map or bank collision!");

  property p_sparse_wr_ok;
    @(posedge clk) disable iff (!rst_n) sparse_wr_ok;
  endproperty
  assert property (p_sparse_wr_ok)
    else $error("ZVT_MT: write port address outside sparse bank map or bank collision!");
  /* verilator lint_on SYNCASYNCNET */
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
