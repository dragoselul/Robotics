function P = computeP(m, COM_z)

    % function computes the potential energy of the link given its COM
    % position (vertical in z0x)
    % COM_z is a single 1x1 point in Z

    P = m * 9.81 * COM_z;

end