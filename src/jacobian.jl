# jacobian (x1, x2) => Jacobiana aplicada em x
# f1'x1 = constant
# f1'x2 = constant
# f2'x1 = 2x1
# f2'x2 = 2x2
# J = [a,   b]
#     [2x1, 2x2]
function jacobian_lc(a::Float32, b::Float32, x1::Float32, x2::Float32)
    J::Matrix(Float32) = zeros(2, 2)
    J = [a b; (2 * x1) (2 * x2)]
    return J
end

function jacobian_central_diff(F, x::AbstractVector{T}; h::Real=1e-7) where T
    n = length(x)
    fx = F(x)
    m = length(fx)
    
    # Aloca a matriz resultante m × n
    J = Matrix{Float64}(undef, m, n)
    
    # Vetores de perturbação
    x_forward = copy(x)
    x_backward = copy(x)
    
    for j in 1:n
        # Perturba apenas a variável j
        x_forward[j] += h
        x_backward[j] -= h
        
        # Diferença central para a coluna j
        J[:, j] = (F(x_forward) - F(x_backward)) / (2 * h)
        
        # Restaura os valores originais
        x_forward[j] = x[j]
        x_backward[j] = x[j]
    end
    
    return J
end

# E.g of test: F(x, y) = [x^2 + y^2 - 1, x - y]
# Jacobiana teórica: [2x  2y;  1  -1]
f(v) = [v[1]^2 + v[2]^2 - 1.0, v[1] - v[2]]

ponto = [1.0, 2.0]
J_num = jacobian_central_diff(f, ponto)

println("Jacobiana Numérica no ponto [1.0, 2.0]:")
display(J_num)
