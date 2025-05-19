function W = waypoints(w)
    n = length(w);  % number of waypoints

    W = zeros(1, 8*(n - 1));  % total constraints for 8th order polynomial segments

    % 1. Position constraints
    idx = 1;
    W(idx) = w(1);
    idx = idx + 1;

    for i = 2:n
        W(idx) = w(i);  % segment end
        W(idx + 1) = w(i);  % next segment start (same waypoint)
        idx = idx + 2;
    end

    % Fix the last value (remove duplicate at the end)
    W(2*n - 1) = 0;  % remove one duplicate
end
% function W = waypoints(w)
%     n = size(w,2);
% 
%     W = zeros(1, 6*(n-1));
%     W(1) = w(1);
%     tmp_i = 2;
% 
%     for i = 2:n
%         W(tmp_i) = w(i);
%         W(tmp_i+1) = w(i);
%         tmp_i = tmp_i + 2;
% 	end
% 	W(2*n-1) = 0;
% end