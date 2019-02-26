function likelihood = likelihoods( i,j )
%summary of the table

N=100;
if i==j && N>i && i>1
    likelihood=0.3;
elseif i==j+1 && N>i && i>2 && j>=2
    likelihood=0.5;
elseif i==j-1 && N-1>i && i>1 && j>=N-1
    likelihood=0.2;
elseif i==1 && j==1
    likelihood=0.1;
elseif i==N && j==N
    likelihood=0.1;
elseif i==2 && j==1
    likelihood=0.9;
elseif i==N-1 && j==N
    likelihood=0.9;
else
    likelihood=0;
end
end

