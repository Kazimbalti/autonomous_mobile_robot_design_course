%%  Run Edge Detection with Code Generation and Without
%
%   Runs edge detection with sober using code generator or edge function in
%   matlab

%%  With Code Generation 
codegen sobel
%%
im = imread('mav.jpg');
im_gray = (0.2989 * double(im(:,:,1)) + 0.5870 * double(im(:,:,2)) + 0.1140 * double(im(:,:,3)))/255;

edgeIm = sobel_mex(im_gray, 0.7);

im_sobel_codegen = repmat(edgeIm, [1 1 3]);

imshowpair(im,im_sobel_codegen,'montage');

%%  Without

im_sobel_edge = edge(im_gray,'sobel');
imshowpair(im,im_sobel_edge,'montage');