function [HTA, HTB, STF, LTF, DTF, DTT, LTT, WTS, HTC, LBL, WBL, HBL, H, LD, LC, LB, LA, LX, LZ] = ReadParameters(filename)
    % read necessary parameters from a file
    try
        tfid = fopen(filename);
        tdata = textscan(tfid, '%s=%s');
        fclose(tfid);
        if( numel(tdata{1}) ~= numel(tdata{2}))
            disp('Error reading file. Missing = !')
            clear tdata tfid
        else
            ndata={ tdata{1} repmat('=', size(tdata{1})) tdata{2}};
            sdata=strcat(ndata{1},ndata{2},ndata{3});
            for i=1:numel(sdata)
                try
                    eval(sdata{i});
                catch
                    sprintf('Bad format in line %d of data file!',i)
                end
            end
            clear i tfid ndata sdata
        end
    catch
        disp('Cannot open file.')
    end

    % default values
    HTA=804;
    HTB=704;
    STF=219;
    LTF=1604;
    DTF=433;
    DTT=691;
    LTT=1104;
    WTS=600;
    HTC=804;
    LBL=150;
    WBL=40;
    HBL=50;
    H=1223;
    LD=194;
    LC=335;
    LB=340;
    LA=295;
    LX=125;
    LZ=332;

    % alter if present in file
    for i=1:size(tdata{1})
        if tdata{1}{i} == "HTA"
            HTA = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "HTB"
            HTB = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "STF"
            STF = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LTF"
            LTF = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "DTF"
            DTF = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "DTT"
            DTT = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LTT"
            LTT = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "WTS"
            WTS = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "HTC"
            HTC = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LBL"
            LBL = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "WBL"
            WBL = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "HBL"
            HBL = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "H"
            H = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LD"
            LD = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LC"
            LC = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LB"
            LB = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LA"
            LA = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LX"
            LX = str2double(tdata{2}{i});
        elseif tdata{1}{i} == "LZ"
            LZ = str2double(tdata{2}{i});
        end
    end
end
