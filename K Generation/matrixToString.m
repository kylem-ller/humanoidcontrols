function str = matrixToString(var)
    s = size(var);
    str = "const float " + inputname(1);
    str = str + strrep(mat2str(flip(s)), ' ', '][') + " = ";
    str = str + repelem('{', length(s));
    for i = 1:numel(var)
        i2 = i -1;
        for l = s
            if mod(i2, l) == 0 && i ~= 1
                str = str + '}{';
            end
            i2 = i2 / l;
        end
        str = str + var(i) + ', ';
    end
    str = convertStringsToChars(str);
    str = str(1:end-2);
    str = convertCharsToStrings(str);
    str = str + repelem('}', length(s));
    str = strrep(strrep(strrep(str, ', }{', '}{'), '{}', '}, {'), '}{', '}, {') + ";";
end