function uploadToArduino(K, T1, T2)
    file = fullfile(pwd,'Arduino','Main','K.h');
    fileID = fopen(file,'w');

    str = matrixToString(K) + "\n";
    if (T1 ~= 0)
        str = str + matrixToString(T1) + "\n";
    end
    if (T2 ~= 0)
        str = str + matrixToString(T2) + "\n";
    end

    fprintf(fileID, str);
    fclose(fileID);
end

