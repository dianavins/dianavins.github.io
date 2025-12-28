---
title: Home
nav_order: 1
---

# HiAER Spike Hardware Docs

Welcome 👋

This documentation teaches hs_api experts and computational neuroscientists how the hardware behind hs_api works. HiAER Spike is built on FPGA technology, which is programmable via hardware language (Verilog) and communicates with the host system (CPU) via hs_bridge. Rather than walking through each file, this documentation teaches from the top down:

## [**Introduction**](introduction) shows a basic hs_api Python implementation for a tiny, 2 layer network.
We will learn everything in the context of this network so that each component is grounded in an example.
- The [Python file](0_Introduction/python_page) (1) defines and compiles the network (2) runs 10 timesteps of inputs on the network
- Has a [page](0_Introduction/hardware_component_definitions) introducing all our hardware components, how they're connected, units of communication, capabilities
- Has a [map](0_Introduction/hardware_map) visualizing the hardware components and connections

## [**Chapter 1**](chapter_1) = network compilation; shows you how the network looks in hardware, no moving parts.
[**1.1**](1_Initializing_the_Network/Chapter_1_1) Explains the locations of everything conceptually, including a visualization  
[**1.2**](1_Initializing_the_Network/Chapter_1_2) Explains how the network was written into the FPGA from the host using hs_bridge and Verilog code

## [**Chapter 2**](chapter_2) = network processing timesteps
(each network timestep consists of phases 0, 1, and 2)  
[**2.1**](2_The_Network_Comes_to_Life/Chapter_2_1) Walks through phases 0, 1 and 2 *conceptually*, no code. Introduces FPGA/Verilog modules as black   boxes.
[**2.2**](2_The_Network_Comes_to_Life/Chapter_2_2) Walks through phases 0, 1 and 2 in Verilog code; beginning to explain the FPGA/Verilog modules.

## [**Chapter 3**](chapter_3) = Verilog file/module breakdown
Each page of Chapter 3 is dedicated to a Verilog file. The first half of the page is essentially a README for the file, followed by Verilog file itself (heavily commented)

## [**Chapter 4**](chapter_4) = RSTDP Implementation
R-STDP is a form of biologically inspired supervised weight update rule ([What is RSTDP?](4_Implementing_RSTDP/what_is_rstdp)). To implement it, we need to know ...
- how to update weights:
  - [**Reading and Writing Synapses**](4_Implementing_RSTDP/synapse_read_write) explains how the pre-existing hs_bridge code for read synapse weights and writing synapse weights works.
- how to implement coincidence STDP:
  - (not yet written)
- how to implement the eligibility trace:
  - (not yet written)


## [**Supplementary Information**](supplementary_information)
Includes:
- [**Appendix**](supplementary_information/appendix) = long list of key terms throughout the whole documentation, thoroughly explained for hardware beginners
- [**Packet Encoding Explained**](supplementary_information/packet_encoding) = explanation of how PCIe TLP packets work for Host-to-FPGA and FPGA-to-Host communication
- [**Addresses Explained**](supplementary_information/addresses_explained) = explanation of how addresses are encoded/
- **Bit Encoding** = explanation of how information is standardly encoded for hardware in hex code
