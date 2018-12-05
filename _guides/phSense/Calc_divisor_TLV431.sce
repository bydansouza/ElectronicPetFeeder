//Divisor de Tensão
//
// Vo = (R2/(R2+R1))*Vi

R1 = 10E+3;
Vo = 0.512
Vi = 1.24

// Vo*R2+Vo*R1 = R2*Vi
//R2*(Vi-Vo) = Vo*R1
R2 = (Vo*R1)/(Vi-Vo)
