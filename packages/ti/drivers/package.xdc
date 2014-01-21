/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 *  ======== ti.drivers ========
 *  TI-RTOS Driver common headers
 *
 *  This is a package that serves as a container for common header files
 *  for various driver modules supplied with the TI-RTOS product.
 *
 *  @p(html)
 *  Documentation for all runtime APIs, instance configuration parameters,
 *  error codes macros and type definitions available to the application
 *  integrator can be found in the
 *  <A HREF="../../../doxygen/html/index.html">Doxygen documentation</A>
 *  for the TI-RTOS product.
 *  @p
 *
 *  The following table shows a list of all driver modules.
 *
 *  @p(html)
 *  <TABLE BORDER="1">
 *  <COLGROUP STYLE="font-weight: bold; color: rgb(0,127,102);"></COLGROUP>
 *  <COLGROUP></COLGROUP>
 *  <COLGROUP></COLGROUP>
 *  <TR>
 *      <TD>WiFi</TD>
 *      <TD>WiFi Driver</TD>
 *      <TD><A HREF="../../../doxygen/html/_wi_fi_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>EMAC</TD>
 *      <TD>EMAC Driver</TD>
 *      <TD><A HREF="../../../doxygen/html/_e_m_a_c_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>GPIO</TD>
 *      <TD>GPIO Manager</TD>
 *      <TD><A HREF="../../../doxygen/html/_g_p_i_o_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>I2C</TD>
 *      <TD>I2C Driver</TD>
 *      <TD><A HREF="../../../doxygen/html/_i2_c_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>SDSPI</TD>
 *      <TD>SDSPI Manager</TD>
 *      <TD><A HREF="../../../doxygen/html/_s_d_s_p_i_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>SPI</TD>
 *      <TD>SPI Manager</TD>
 *      <TD><A HREF="../../../doxygen/html/_s_p_i_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>UART</TD>
 *      <TD>UART Manager</TD>
 *      <TD><A HREF="../../../doxygen/html/_u_a_r_t_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>USBMSCHFatFs</TD>
 *      <TD>USBMSCHFatFs Manager</TD>
 *      <TD><A HREF="../../../doxygen/html/_u_s_b_m_s_c_h_fat_fs_8h.html">Doxygen</A></TD>
 *  </TR>
 *  <TR>
 *      <TD>Watchdog</TD>
 *      <TD>Watchdog Manager</TD>
 *      <TD><A HREF="../../../doxygen/html/_watchdog_8h.html">Doxygen</A></TD>
 *  </TR>
 *
 *  </TABLE>
 *  @p
 *
 */
package ti.drivers [1,0,0] {
    module EMAC;
    module SDSPI;
    module I2C;
    module USBMSCHFatFs;
    module GPIO;
    module UART;
    module SPI;
    module WiFi;
    module Watchdog;
};
