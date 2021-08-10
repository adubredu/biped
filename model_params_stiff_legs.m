function [r,m,M_H, M_T, l,g,p_st_foot, Mtotal] = model_params_stiff_legs

r=1;
m=5;
M_H=15;
M_T=10;
l=0.5;
g=9.8;
Mtotal = m*2+M_H+M_T;

p_stx_foot_o = 0;
p_sty_foot_o = 0;
p_st_foot = [p_stx_foot_o; p_sty_foot_o];