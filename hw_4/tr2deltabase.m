function delta_base = tr2deltabase(T_cur, T_des)

    delta = tr2delta(T_cur, T_des);
    
    % pull of the rotation matrix part
    R2base = [T_cur.n, T_cur.o, T_cur.a];
    
    delta_base = [R2base, zeros(3); zeros(3), R2base] * delta;
    
end