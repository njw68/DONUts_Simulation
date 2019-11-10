%Reconstruct
function path = Reconstruct(path,visited)
visited_parent = [];
for i=1:length(visited)
    visited_parent(end+1) = visited.parent;
end
while path(1).parent ~= 0
    for i=1:length(visited)
        if visited(i).state == path(1).parent
            path = [visited(i),path];
            i=length(visited)+1;
        end
    end
end

path(1) = [];


end