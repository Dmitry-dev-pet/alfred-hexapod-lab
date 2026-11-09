// MuJoCo coordinates: z is up. Keep the viewer outside solid terrain.
export function cameraOutsideTerrain(position,terrain,margin=.012){
 const [x,y,z]=position;const out=[x,y,Math.max(z,margin)];
 if(!terrain?.height_m)return out;
 const [cx,cy]=terrain.center_xy_m,[hx,hy]=terrain.half_size_xy_m;
 if(x>=cx-hx-margin&&x<=cx+hx+margin&&y>=cy-hy-margin&&y<=cy+hy+margin&&out[2]<terrain.height_m+margin)out[2]=terrain.height_m+margin;
 return out;
}
