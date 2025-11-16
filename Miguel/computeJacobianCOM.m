function full_jacobian = computeJacobianCOM(T_COMlist, T_list, link_n)
%#codegen
    % Always 6x4, i.e. max 4 links
    full_jacobian = zeros(6, 4);

    for i = 1:4
        if i <= link_n
            T0x_COM = T_COMlist(:, :, i);
            T0x     = T_list(:, :, i);
            z       = T0x(1:3, 3);
            COM     = T0x_COM(1:3, 4);
            origin_rotation = T0x(1:3, 4);

            column_J = Jvw(z, COM, origin_rotation);  % 6x1
            full_jacobian(:, i) = column_J;
        else
            % Can leave as zeros (already is), or explicitly:
            % full_jacobian(:, i) = zeros(6,1);
        end
    end
end
