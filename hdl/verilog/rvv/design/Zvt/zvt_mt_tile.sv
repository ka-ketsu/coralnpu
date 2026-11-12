// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`ifndef HDL_VERILOG_RVV_DESIGN_RVV_SVH
`include "rvv_backend.svh"
`endif

// -----------------------------------------------------------------------------
// zvt_mt_tile: 4-KB hierarchical tile unit for bottom-up ASIC synthesis & P&R
//
// Groups 16 identical zvt_mt_subtile banks (32,768 flip-flops total) into a
// single reusable ~4.6K-pin block instantiated 16 times in zvt_mt.
// -----------------------------------------------------------------------------
module zvt_mt_tile (
  input  logic                                   clk,
  input  logic                                   rst_n,
  input  logic [15:0][3:0]                       rrow,
  output logic [15:0][`SUBTILE_SIZE*8-1:0]       rdata,
  output logic [`SUBTILE_SIZE*8-1:0]             row0_rdata,
`ifdef RVVI_ON
  output logic [15:0][15:0][`SUBTILE_SIZE*8-1:0] rvvi_tile_rows,
`endif
  input  logic [15:0][`SUBTILE_SIZE-1:0]         wen,
  input  logic [15:0][3:0]                       wrow,
  input  logic [15:0][`SUBTILE_SIZE*8-1:0]       wdata
);

  /* verilator lint_off UNUSEDSIGNAL */
  logic [15:0][`SUBTILE_SIZE*8-1:0] sub_row0_rdata;
  /* verilator lint_on UNUSEDSIGNAL */

  assign row0_rdata = sub_row0_rdata[0];

  genvar u_g;
  generate
    for (u_g = 0; u_g < 16; u_g++) begin : gen_subtile
      zvt_mt_subtile u_subtile (
        .clk        (clk),
        .rst_n      (rst_n),
        .rrow       (rrow[u_g]),
        .rdata      (rdata[u_g]),
        .row0_rdata (sub_row0_rdata[u_g]),
      `ifdef RVVI_ON
        .rvvi_rows  (rvvi_tile_rows[u_g]),
      `endif
        .wen        (wen[u_g]),
        .wrow       (wrow[u_g]),
        .wdata      (wdata[u_g])
      );
    end
  endgenerate

endmodule
