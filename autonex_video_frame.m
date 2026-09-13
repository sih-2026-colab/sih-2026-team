function canvas=autonex_video_frame(fig)
% Fixed even video dimensions despite desktop DPI or figure capture size changes.
captured=getframe(fig); pixels=captured.cdata;
height=720; width=1280;
scale=min(width/size(pixels,2),height/size(pixels,1));
pixels=imresize(pixels,scale);
canvas=zeros(height,width,3,'uint8');
row=floor((height-size(pixels,1))/2)+1;
col=floor((width-size(pixels,2))/2)+1;
canvas(row:row+size(pixels,1)-1,col:col+size(pixels,2)-1,:)=pixels;
end
