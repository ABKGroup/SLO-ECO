
set clk_period 5.4
create_clock [get_ports clk_i] -name CLK -period $clk_period
set io_delay [expr $clk_period * .2]
set_input_delay -clock CLK $io_delay [get_ports reset_i]
set_input_delay -clock CLK $io_delay [get_ports {icache_id_i[0]}]
set_input_delay -clock CLK $io_delay [get_ports {bp_fe_cmd_i[108:0]}]
set_input_delay -clock CLK $io_delay [get_ports bp_fe_cmd_v_i]
set_input_delay -clock CLK $io_delay [get_ports bp_fe_queue_ready_i]
set_input_delay -clock CLK $io_delay [get_ports lce_cce_req_ready_i]
set_input_delay -clock CLK $io_delay [get_ports lce_cce_resp_ready_i]
set_input_delay -clock CLK $io_delay [get_ports lce_cce_data_resp_ready_i]
set_input_delay -clock CLK $io_delay [get_ports {cce_lce_cmd_i[35:0]}]
set_input_delay -clock CLK $io_delay [get_ports cce_lce_cmd_v_i]
set_input_delay -clock CLK $io_delay [get_ports {cce_lce_data_cmd_i[539:0]}]
set_input_delay -clock CLK $io_delay [get_ports cce_lce_data_cmd_v_i]
set_input_delay -clock CLK $io_delay [get_ports {lce_lce_tr_resp_i[538:0]}]
set_input_delay -clock CLK $io_delay [get_ports lce_lce_tr_resp_v_i]
set_input_delay -clock CLK $io_delay [get_ports lce_lce_tr_resp_ready_i]
set_output_delay -clock CLK $io_delay [get_ports bp_fe_cmd_ready_o]
set_output_delay -clock CLK $io_delay [get_ports {bp_fe_queue_o[133:0]}]
set_output_delay -clock CLK $io_delay [get_ports bp_fe_queue_v_o]
set_output_delay -clock CLK $io_delay [get_ports {lce_cce_req_o[29:0]}]
set_output_delay -clock CLK $io_delay [get_ports lce_cce_req_v_o]
set_output_delay -clock CLK $io_delay [get_ports {lce_cce_resp_o[25:0]}]
set_output_delay -clock CLK $io_delay [get_ports lce_cce_resp_v_o]
set_output_delay -clock CLK $io_delay [get_ports {lce_cce_data_resp_o[536:0]}]
set_output_delay -clock CLK $io_delay [get_ports lce_cce_data_resp_v_o]
set_output_delay -clock CLK $io_delay [get_ports cce_lce_cmd_ready_o]
set_output_delay -clock CLK $io_delay [get_ports cce_lce_data_cmd_ready_o]
set_output_delay -clock CLK $io_delay [get_ports lce_lce_tr_resp_ready_o]
set_output_delay -clock CLK $io_delay [get_ports {lce_lce_tr_resp_o[538:0]}]
set_output_delay -clock CLK $io_delay [get_ports lce_lce_tr_resp_v_o]
