from sympy import *
from sympy import symbols
from sympy.interactive.printing import init_printing
init_printing(use_unicode=False, wrap_line=False)
from sympy.matrices import Matrix, eye, zeros, ones, diag, GramSchmidt
from sympy import collect, sympify, Wild
from sympy.printing import print_ccode


x, y, A, B, C, Tx, Ty, Ty, Tz, Px, Py, Bx, By, Zhome = symbols("x y A B C Tx Ty Ty Tz Px Py Bx By Zhome")

P =    Matrix([[Px, Py, 0, 1]])

Rx =   Matrix([[      1,       0,       0,       0],
               [      0,  cos(A),  sin(A),       0],
               [      0, -sin(A),  cos(A),       0],
               [      0,       0,       0,       1]])

Ry =   Matrix([[ cos(B),       0, -sin(B),       0],
               [      0,       1,       0,       0],
               [ sin(B),       0,  cos(B),       0],
               [      0,       0,       0,       1]])

Rz =   Matrix([[ cos(C),  sin(C),       0,       0],
               [-sin(C),  cos(C),       0,       0],
               [      0,       0,       1,       0],
               [      0,       0,       0,       1]])

Txyz = Matrix([[      1,       0,       0,       0],
               [      0,       1,       0,       0],
               [      0,       0,       1,       0],
               [     Tx,      Ty,      Tz,       1]])

TB =   Matrix([[      1,       0,       0,       0],
               [      0,       1,       0,       0],
               [      0,       0,       1,       0],
               [    -Bx,     -By,  -Zhome,       1]])

BP = P * Rx * Ry * Rz * Txyz * TB

print("BP = ")
for elem in BP:
    print(elem)
print("")


init_printing()
x, y, z, a, b, c, phi, ROD_LENGTH, ARM_LENGTH, theta_s = symbols("x y z a b c phi ROD_LENGTH ARM_LENGTH theta_s")

# Eq du cercle paramétrique
sol = solve(((ARM_LENGTH*cos(phi)-a)**2 + b**2 + (ARM_LENGTH*sin(phi)-c)**2 - ROD_LENGTH**2), [phi],
     force=True, manual=True, set=True)

cpt = 0
for elem in sol[1]:
    print(f"\n\nsolution {cpt}")
    cpt += 1
    print_ccode(simplify(elem[0]), standard='C99')


# Eq du cercle carthésienne
sol = solve(((sqrt(ARM_LENGTH**2-z**2)-a)**2 + b**2 + (sqrt(ARM_LENGTH**2-x**2)-c)**2 - ROD_LENGTH**2), [x, y],
     force=True, manual=True, set=True)

cpt = 0
print(sol)
print(f"{sol[0]}\n")
for elem in sol[1]:
    print(f"solution {cpt}\n\n")
    cpt += 1
    print_ccode(simplify(elem[0]), standard='C99')
