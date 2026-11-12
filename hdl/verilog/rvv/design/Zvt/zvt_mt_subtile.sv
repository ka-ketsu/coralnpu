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
// zvt_mt_subtile: 256-Byte logical bank unit for bottom-up ASIC synthesis
//
// Encapsulates 16 rows x 16 bytes (2,048 flip-flops) of edff storage together
// with its local 4-to-16 write-row byte-enable decoder and 16-to-1 128-bit
// read-row multiplexer, keeping boundary pin count compact (~410 pins).
// -----------------------------------------------------------------------------
module zvt_mt_subtile (
  input  logic                             clk,
  input  logic                             rst_n,
  input  logic [3:0]                       rrow,
  output logic [`SUBTILE_SIZE*8-1:0]       rdata,
  output logic [`SUBTILE_SIZE*8-1:0]       row0_rdata,
`ifdef RVVI_ON
  output logic [15:0][`SUBTILE_SIZE*8-1:0] rvvi_rows,
`endif
  input  logic [`SUBTILE_SIZE-1:0]         wen,
  input  logic [3:0]                       wrow,
  input  logic [`SUBTILE_SIZE*8-1:0]       wdata
);

  localparam int unsigned NUM_ROW = 32'd16;

  logic [15:0][`SUBTILE_SIZE-1:0]   row_wen;
  logic [15:0][`SUBTILE_SIZE*8-1:0] row_q;

  always_comb begin
    row_wen = '0;
    rdata   = '0;
    for (int unsigned r = 32'd0; r < NUM_ROW; r = r + 32'd1) begin
      if (wrow == r[3:0]) begin
        row_wen[r] = wen;
      end
      if (rrow == r[3:0]) begin
        rdata = row_q[r];
      end
    end
  end

  assign row0_rdata = row_q[0];
`ifdef RVVI_ON
  assign rvvi_rows  = row_q;
`endif

  genvar r_g, h_g;
  generate
    for (r_g = 0; r_g < 16; r_g++) begin : gen_row
      for (h_g = 0; h_g < `SUBTILE_SIZE; h_g++) begin : gen_byte
        edff #(
          .T (logic [`BYTE_WIDTH-1:0])
        ) u_edff_byte (
          .q     (row_q[r_g][h_g*`BYTE_WIDTH +: `BYTE_WIDTH]),
          .e     (row_wen[r_g][h_g]),
          .d     (wdata[h_g*`BYTE_WIDTH +: `BYTE_WIDTH]),
          .clk   (clk),
          .rst_n (rst_n)
        );
      end
    end
  endgenerate

`ifdef ASSERT_ON
  assert property (@(posedge clk) disable iff (!rst_n) !$isunknown(row_q))
    else $error("MTSUBTILE: unknown data detected in subtile storage");
`endif

endmodule
