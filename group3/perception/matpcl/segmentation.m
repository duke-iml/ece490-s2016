clear
while 1
    %watch wether have inputfile
    done = 0;
    while 1
        if exist('chenyugo.txt', 'file')
            fileID = fopen('chenyugo.txt','r+');
            contents = fscanf(fileID,'%s')
            fprintf(fileID,'%s','');
            fclose(fileID);
            if strcmp(contents,'chenyugo')
                break;
            end
            pause(1)
            
        end
        
    end
    %     while (or(~exist('chenyugo.txt', 'fil
    %e'), ~exist('cloud.mat', 'file')))
    %         pause(1)
    %         if exist('end.txt', 'file')
    %             break;
    %         end
    %     end
    %     starting_loop = 0
    %     rehash
    %     clear
    %     if exist('end.txt', 'file')
    %         break;
    %     end
    delete('*.txt')
    delete('seg*.mat')
    clear
    load('cloud.mat')
    
    pause(0.5)
    
    r=[];
    o=1;   %indicate how many points are searched
    oo=1;  %indicate how many iritations are done
    ooo=1; %indicate how many areas are found
    %overlap=0;
    cloud(find(isnan(cloud)))=0;
    cloud( ~any(cloud,2), : ) = []; %delete the all zero point
    cloudcopy=cloud;
    cloud = cloud(:,1:3);
    figure(80)
    pcshow(cloud)
    ns1=createns(cloud,'nsmethod','kdtree');
    [idx1,dist]=knnsearch(ns1,cloud,'k',20); %kd-tree algorithm
    dmin=floor(0.5*length(cloud));  %number of start point
    %dmin=600;
    %dmin=500;
    list=[dmin]; % the index number of points in each iritation
    sellist=[]; % selected points for one are
    dist( ~any(dist,2), : ) = [];
    edthall=median(median(dist)); %global median of distance
    
    leftpoint(1,1:length(cloud))=1; %indicate whether points are selected
    listall={mat2cell(list,1,1)};
    while abs(max(max(leftpoint)))>0 && ooo<=10
        while  numel(list)>0 && oo<=5000
            
            
            list=cell2mat(listall{oo});
            for k=1:length(list)
                dmin=list(1);
                list(1)=[];
                edth=median(dist(dmin,:));
                
                [n,v,p]=svd([cloud(idx1(dmin,:),1) cloud(idx1(dmin,:),2) cloud(idx1(dmin,:),3)]);
                dminmean=mean([cloud(idx1(dmin,:),1) cloud(idx1(dmin,:),2) cloud(idx1(dmin,:),3)],1);
                if length(cloud) < 30
                    break;
                end
                for i=1:20
                    odth1(i)=abs(dot([(cloud(idx1(dmin,i),1)-dminmean(1)),(cloud(idx1(dmin,i),2)-dminmean(2)),(cloud(idx1(dmin,i),3)-dminmean(3))],p(:,3)));
                end
                odth=median(odth1);
                pointss=[];
                for i=1:20
                    if dist(dmin,i)<=edth && dist(dmin,i)<=edthall && dot([(cloud(idx1(dmin,i),1)-dminmean(1)),(cloud(idx1(dmin,i),2)-dminmean(2)),(cloud(idx1(dmin,i),3)-dminmean(3))],p(:,3))<=odth
                        
                        pointss(i,:)=cloudcopy(idx1(dmin,i),:);
                        pointnum(i)=idx1(dmin,i);
                        leftpoint(1,idx1(dmin,i))=0;
                        cloud(idx1(dmin,i),1:3)=0;
                        cloudcopy(idx1(dmin,i),:)=0;
                        
                    end
                end
                
                r=[r;pointss];
                pointnum1=pointnum';
                pointnum1(~any(pointnum1,2),: ) = [];
                for i=1:length(pointnum1)
                    if find(list==pointnum1(i))
                        if find(sellist==pointnum1(i))
                            list(find(list==pointnum1(i)))=[];
                            pointnum1(i)=0;
                        end
                    end
                    if find(sellist==pointnum1(i))
                        list(find(list==pointnum1(i)))=[];
                        pointnum1(i)=0;
                    end
                    if pointnum1(1) == dmin
                        pointnum1(1)=0;
                    end
                end
                pointnum1(find(pointnum1==0))=[];
                list=[list;pointnum1];
                list( ~any(list,2), : ) = [];
                %dmin=list(1);
                %list(1)=[];
                if find(sellist==dmin)
                    sellist=sellist;
                else
                    sellist=[sellist;dmin];
                end
                
                o=o+1;
                if isempty(list)
                    break
                end
                pointnum=[];
                pointnum1=[];
                pointss=[];
            end
            
            if isempty(list)
                break
            else
                oo=oo+1;
                listall{oo}=mat2cell(list,length(list),1);
            end
        end
        
        r( ~any(r,2), : ) = [];
        if size(r,1)>100
            name = strcat('seg',int2str(ooo))
            save(name,'r');
            figure(ooo)
            pcshow(r(:,1:3))
            %{
    sellistpoint(:,1:3)=pointdownstore(sellist(:,1),:);
    ns2=createns(point1,'nsmethod','kdtree');
    [idx2,dist2]=knnsearch(point1,sellistpoint,'k',20);
    for i=1:length(sellist)
        for j=1:20
        pointttt(i+(j-1)*20,:)=point1(idx2(i,j),:);
        end
        end
        figure(10)
        pcshow(pointttt)
            %}
        end
        
        pointnum=[];
        pointnum1=[];
        pointss=[];
        r=[];
        o=1;
        oo=1;
        
        
        cloudcopy( ~any(cloud,2), : ) = [];
        cloud( ~any(cloud,2), : ) = [];
        ns1=createns(cloud,'nsmethod','kdtree');
        [idx1,dist]=knnsearch(ns1,cloud,'k',20);
        leftpoint(1,length(cloud))=1;
        if length(cloud) <30
            break;
        end
        dmin=floor(0.5*length(cloud));
        list=[dmin]; 
        sellist=[dmin];
        edthall=median(median(dist));
        listall={mat2cell(list,1,1)};
        ooo=ooo+1;
        
    end
    save('chenyudone.txt','ooo')
    rehash
    if exist('end.txt', 'file')
        break;
    end
end