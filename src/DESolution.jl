# jacobian (x1, x2) => Jacobiana aplicada em x
# f1'x1 = constant
# f1'x2 = constant
# f2'x1 = 2x1
# f2'x2 = 2x2
# J = [a,   b]
#     [2x1, 2x2]


# Predictor (assume constant)
function jacobian_lc(a::Float32, b::Float32, x1::Float32, x2::Float32)
    J::Matrix(Float32) = zeros(2, 2)
    J = [a b; (2 * x1) (2 * x2)]
    return J
end

# Inverse Jacobian

# Newton correction
function newton_step_lc(inv_j::Matrix(Float32), pn::Vecotr(Float32), a::Float32, b::Float32, c::Float32, d1::Float32, d2::Float32)
    @assert(length(pn) == 2)

    return pn - inv_j * [(a * pn[1] + b*pn[2] + c) ()]
end