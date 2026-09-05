function write_json(var, filename)
%WRITE_JSON Encode a MATLAB variable to JSON and save to file
%   write_json(var, filename)
    try
        j = jsonencode(var);
        fid = fopen(filename, 'w');
        if fid == -1
            error('Could not open file for writing: %s', filename);
        end
        fwrite(fid, j, 'char');
        fclose(fid);
    catch ME
        warning('Failed to write JSON %s: %s', filename, ME.message);
    end
end
