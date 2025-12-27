---
title: Introduction
has_children: true
nav_order: 2
---

# Introduction

This documentation teaches the low-level hardware implementation of hs_api to software experts and computational neuroscientists. While hs_api provides a high-level Python interface for creating spiking neural networks, understanding what happens beneath the surface—how your network definition translates into transistor-level operations on FPGA hardware, how data flows through PCIe transactions, how neurons are stored in DRAM cells and updated at nanosecond timescales—is essential for optimizing performance, debugging behavior, and designing networks that leverage the unique capabilities of neuromorphic hardware. We'll build this understanding from the ground up, starting with a simple network example and tracing its journey from Python code through software layers, across physical communication buses, into programmable logic gates, and finally to individual memory cells storing neuron states.