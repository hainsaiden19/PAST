function mu_M = Magnetorquer(current, n, A)


moment = n*A*current; % L M N
% caps moment at 2 A m^2
% for i = 1:length(moment)
%     mom = moment(i);
%     if mom > 2
%         moment(i) = 2;
%     end
% end     

mu_M = moment;