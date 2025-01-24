
// import sympy
// from sympy import latex
// from sympy.physics.mechanics import init_vprinting
// from IPython.display import display, Math

// init_vprinting()
// x, y, z, pi, lam, u1, u2, u3 = sympy.symbols('x y z \\pi \\lambda u1 u2 u3')
// xyz = sympy.Matrix([x, y, z])
// u = sympy.Matrix([u1, u2, u3])

// w = sympy.exp(-1j * 2*pi/lam * u.dot(xyz))
// dw_du1 = sympy.diff(w, u1)
// dw_du2 = sympy.diff(w, u2)
// dw_du3 = sympy.diff(w, u3)

// display(Math( "w = " + latex(w) ))
// display(Math('h(w,u_1) = ' + sympy.latex(dw_du1)))
// display(Math('h(w,u_2) = ' + sympy.latex(dw_du2)))
// display(Math('h(w,u_3) = ' + sympy.latex(dw_du3)))

int main() {
  return 0;
}