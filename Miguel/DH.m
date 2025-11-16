function transformation = DH(theta, dz, dx, alpha)
    
    transformation = rotationZ(theta) * translationMatrix(0, 0, dz) * translationMatrix(dx, 0, 0) * rotationX(alpha);

end