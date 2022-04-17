function T = GetTransformMatrix(xTranslation, yTranslation, yaw)
    Rp_rotate = makehgtform('zrotate', yaw);
    Rp_trans  = makehgtform('translate',[xTranslation yTranslation 0]);
    T = Rp_trans*Rp_rotate;
end