%
% findloop.m
% -- script to find ordered indices from x,y scatter data to form a loop
%
%
% Ian Stavness -- 25/01/2010

function k = findloop(x,y)
xy = [x,y];
N = length(x);
k = [];
newk = 1;
% while (isempty(find(k==newk))) % while k[] does not contain newk
%     k = [k;newk];
%     % find closest point
%     pnt = repmat([x(newk),y(newk)],N,1);
%     idx = find (1:N~=newk)';
%     dist = sqrt(sum((pnt-xy).^2,2));
%     newk = find(dist==min(dist(idx)));
% end

k = zeros(N+1,1);
idxsleft = 1:N;
newk = 1;
k(1)=newk;
k(N+1)=newk;
for i=2:N
    % find closest point
    pnt = repmat([x(newk),y(newk)],N,1);
    newkidx = find(idxsleft==newk);
    idxsleft(newkidx)=-1; % flag as used
    idx = find(idxsleft~=-1);
    dist = sqrt(sum((pnt-xy).^2,2));
    newk = find(dist==min(dist(idx)));
    k(i)=newk;
end