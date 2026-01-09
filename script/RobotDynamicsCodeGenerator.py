import sympy
import sympybotics
import os
from sympybotics.symcode import code_to_func, Subexprs
import re


class RobotUnifiedDynamicsGenerator:
    def __init__(self):
        self.rbt, self.rbtdef = self.create_robot_model()
        self.generate_all()

    # -------------------------------------------------
    def create_robot_model(self):
        rbtdef = sympybotics.RobotDef(
            "w1",
            [
                ("-pi/2", 0, 0.1025, "q"),
                ("pi/2",  0, 0.0,    "q"),
                ("-pi/2", 0, 0.26,   "q"),
                ("pi/2",  0, 0.0,    "q"),
                ("-pi/2", 0, 0.264,  "q"),
                ("pi/2",  0, 0.0,    "q"),
                (0,       0, 0.0,    "q"),
            ],
            dh_convention="standard",
        )

        rbtdef.frictionmodel = {"Coulomb", "viscous", "offset"}
        rbtdef.driveinertiamodel = "simplified"
        rbtdef.gravityacc = sympy.Matrix([0, 0, 0])

        rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)
        rbt.calc_base_parms()
        return rbt, rbtdef

    # -------------------------------------------------
    def generate_all(self):
        self.generate_M_term_code()

    # -------------------------------------------------

    def generate_M_term_code(self):

        # 构造 tau = H(g) * Pb * dyn_base_parameters（7x1）
        pb_syms = sympy.symbols(f'pb0:{len(self.rbt.dyn.baseparms)}')  # 仅用于构造表达式
        pb = sympy.Matrix(pb_syms)
        M_expr = self.rbt.dyn.H * self.rbt.dyn.Pb * pb  # 7x1
        M_se = Subexprs()
        self.rbt.dyn.gen_regressor(M_se.collect)
        M_term_ode = M_se.get(M_expr)

        q_subs = {q: f'q[{i}]' for i, q in enumerate(self.rbtdef.q)}
        dq_subs = {dq: f'0' for i, dq in enumerate(self.rbtdef.dq)}
        ddq_subs = {ddq: f'ddq[{i}]' for i, ddq in enumerate(self.rbtdef.ddq)}
        pb_subs = {sym: f'dyn_base_params[{i}]' for i, sym in enumerate(pb_syms)}
        subs = {**q_subs, **dq_subs, **ddq_subs, **pb_subs}

        # 生成 C++ 版本（用 C 代码风格，.cpp 可直接编译）
        func_src_c = code_to_func(
            'C', M_term_ode, 'M_term_out', 'tau_M_term',
            ['q', 'ddq', 'dyn_base_params'], subs
        )
        header_cpp = (
            '#include <math.h>\n'
            '#include <stddef.h>\n'
            'static inline double sign(double x){ return (x>0) - (x<0); }\n\n'
        )

        out_path = os.path.join(os.path.dirname(__file__), 'tau_M_term.cpp')
        with open(out_path, 'w') as f:
            f.write(header_cpp + func_src_c)
            print(f'Generated {out_path}')

    def generate_inv_dyn_code(self):
        """ 生成完整逆动力学计算代码，输出 invdyn_tau.cpp 文件"""

        # 构造 tau = H(g) * Pb * dyn_base_parameters（7x1）
        pb_syms = sympy.symbols(f'pb0:{len(self.rbt.dyn.baseparms)}')  # 仅用于构造表达式
        pb = sympy.Matrix(pb_syms)
        tau_expr = self.rbt.dyn.H * self.rbt.dyn.Pb * pb  # 7x1

        # 复用 H 的子表达式，生成 code 结构
        T_se = sympybotics.symcode.Subexprs()
        T_se._subexp_iv = self.rbt._H_se
        tau_code = T_se.get(tau_expr)
        print(tau_code)

        # 构造替换映射与函数签名（包含 dyn_base_params 与 gravity_acc）
        q_subs = {q: f'q[{i}]' for i, q in enumerate(self.rbtdef.q)}
        dq_subs = {dq: f'dq[{i}]' for i, dq in enumerate(self.rbtdef.dq)}
        ddq_subs = {ddq: f'ddq[{i}]' for i, ddq in enumerate(self.rbtdef.ddq)}
        pb_subs = {sym: f'dyn_base_params[{i}]' for i, sym in enumerate(pb_syms)}
        subs = {**q_subs, **dq_subs, **ddq_subs, **pb_subs}

        # 6) 生成 C++ 版本（用 C 代码风格，.cpp 可直接编译）
        func_src_c = code_to_func(
            'C', tau_code, 'tau_out', 'invdyn_tau',
            ['q', 'dq', 'ddq', 'dyn_base_params'], subs
        )
        header_cpp = (
            '#include <math.h>\n'
            '#include <stddef.h>\n'
            'static inline double sign(double x){ return (x>0) - (x<0); }\n\n'
        )

        # 替换 ((x) > 0) - ((x) < 0) 为 sign(x)
        pattern = re.compile(r'\(\(([^()]+?)\) > 0\) - \(\(\1\) < 0\)')
        while True:
            new_func_src_c = pattern.sub(r'sign(\1)', func_src_c)
            if new_func_src_c == func_src_c:
                break
            func_src_c = new_func_src_c

        out_path = os.path.join(os.path.dirname(__file__), 'invdyn_tau.cpp')
        with open(out_path, 'w') as f:
            f.write(header_cpp + func_src_c)
            print(f'Generated {out_path}')

if __name__ == "__main__":
    RobotUnifiedDynamicsGenerator()
