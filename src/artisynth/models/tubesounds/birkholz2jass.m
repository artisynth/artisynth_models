function [area,x,tractLen] = birkholz2jass(fin,N,fnout)
%
% convert the area function data files from Peter Birkholz's 
% speech synthesizer to a format that can be read by the jass
% control panel. N = number of area control points
  
  [area,x,tractLen] = readData(fin);
  % now interpolate on even spaced grid with N points
  xi = (0:N-1)*tractLen/(N-1);
  Ai = interp1(x,area,xi);
  subplot(2,1,1)
  plot(x,area);
  title 'org'
  subplot(2,1,2)
  plot(xi,Ai);
  title 'interp'
  fout = fopen(fnout,'w');
  createJassImport(xi,Ai,tractLen,fout);
  fclose(fout);


function  createJassImport(xi,Ai,tractLen,fout)
  begin.Mstr = 'lip M mult';
  begin.Mval=4.5;
  begin.Mmin=.001;
  begin.Mmax=5;
  begin.dstr = 'lip d mult';
  begin.dval = 1;
  begin.dmin =.05;
  begin.dmax =1000;
  begin.wstr = 'wall damp';
  begin.wval = .02;
  begin.wmin =.0;
  begin.wmax =.1;

  fprintf(fout,'%s\n',begin.Mstr);
  fprintf(fout,'%f\n',begin.Mval);
  fprintf(fout,'%f\n',begin.Mmin);
  fprintf(fout,'%f\n',begin.Mmax);
  fprintf(fout,'%s\n',begin.dstr);
  fprintf(fout,'%f\n',begin.dval);
  fprintf(fout,'%f\n',begin.dmin);
  fprintf(fout,'%f\n',begin.dmax);
  fprintf(fout,'%s\n',begin.wstr);
  fprintf(fout,'%f\n',begin.wval);
  fprintf(fout,'%f\n',begin.wmin);
  fprintf(fout,'%f\n',begin.wmax);
  fprintf(fout,'%s\n','length');
  fprintf(fout,'%f\n',tractLen/100);
  fprintf(fout,'%f\n',.15);
  fprintf(fout,'%f\n',.68);
  
  area.min = .01;
  area.max = 10;
  for k = 1:length(xi);
    fprintf(fout,'A(%d)\n',k-1);
    fprintf(fout,'%f\n',Ai(k));
    fprintf(fout,'%f\n',area.min);
    fprintf(fout,'%f\n',area.max);    
  end

function [area,x,tractLen]=readData(fin)
  
%
% convert Birkholz format area function to jass conrol panel with N area
% sections
  
  fin = fopen(fin,'r');
  rc = 0;
  data_block = 0;
  data_block_old = 0;
  non_numeric_lines_seen = 0;
  arrIndex = 1; 
  area0 = [];
  x0 = [];
  tractLen = 0;
  while(rc ~= -1)
    rc = fgets(fin);
    isNumeric = isstrprop(rc,'digit');
    if(isNumeric(1))
      rc = strrep(rc,',','.');
      y = sscanf(rc,'%f');
      if(data_block == 1)
        x0(arrIndex) = y;     
      elseif(data_block == 2)
        area0(arrIndex) = y;
      end
      arrIndex = arrIndex + 1;
      %fprintf('NUMBER: %f\n',y);            
    else
      %fprintf('ELSE: %s',rc);
      non_numeric_lines_seen =   non_numeric_lines_seen + 1;
      if(non_numeric_lines_seen == 2)
        data_block = 1;
      elseif(non_numeric_lines_seen == 6)
        data_block = 2;
      elseif(non_numeric_lines_seen > 6)
        rc = -1;
      end
      if(data_block_old ~= data_block)
        arrIndex = 1;
        data_block_old = data_block;
      end

    end
  end

  tractLen = x0(end);
  x1 = [x0(1) x0(2:2:end)];
  for k=1:length(x1)-1;
    x(k) =  (x1(k)+x1(k+1))/2;
  end
  x = x - x(1);
  x = x * tractLen/x(end);

  area = area0(2:2:end);
  
  fclose(fin);