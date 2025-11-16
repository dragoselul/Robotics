function vector = computeNumVect(X, t)

    % where X is a vector (row)
    % functions works out its polinomial evaluation like:
    % [a, b, c] = at^2, bt, c
    % t is where the vector is evaluated

    % can handle t^6 t^5 0 0 0 0 vectors (usual case in derivations)
       
    n = length(X);
    exponents = n-1:-1:0;
    vector = X.*t.^exponents;
end