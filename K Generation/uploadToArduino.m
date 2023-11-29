function uploadToArduino(K, H)
    file = fullfile(pwd,'Arduino','Main','K.h');
    fileID = fopen(file,'w');

    str = matrixToString(K) + "\n";
    str = str + matrixToString(H) + "\n";

    fprintf(fileID, str);
    fclose(fileID);
end

