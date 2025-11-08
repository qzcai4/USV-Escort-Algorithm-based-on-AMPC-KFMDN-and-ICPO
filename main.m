clc,clear;
close all;

Configure_environment();

Variable_declaration;
Parameter_setting();
Generate_initial_conditions();

Main_simulation_loop();

Figplot()