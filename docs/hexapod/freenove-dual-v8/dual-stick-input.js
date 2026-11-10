/** Independent two-pointer controls for offline CAD30 viewer.
 * Each side owns a pointerId, so lifting one thumb cannot release the other.
 * Right Y is reserved in walking mode; in body mode it commands pitch.
 */
const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
const VALID_SIDES=new Set(['left','right']);
export function normalizedPad(x,y,deadzone=.10){
  if(!Number.isFinite(x)||!Number.isFinite(y))return {x:0,y:0};
  const r=Math.hypot(x,y);
  if(r<=deadzone)return {x:0,y:0};
  const s=r>1?1/r:1;
  return {x:clamp(x*s,-1,1),y:clamp(y*s,-1,1)};
}
export class DualStickInput {
  constructor(){this.left={pointer:null,x:0,y:0};this.right={pointer:null,x:0,y:0};}
  side(which){if(!VALID_SIDES.has(which))throw Error('Unknown stick side');return this[which];}
  start(which,pointerId,x,y){
    const side=this.side(which);if(side.pointer!==null)return false;
    if(pointerId===null||pointerId===undefined||this.other(which).pointer===pointerId)return false;
    Object.assign(side,{pointer:pointerId,...normalizedPad(x,y)});return true;
  }
  other(which){return this[which==='left'?'right':'left'];}
  move(which,pointerId,x,y){
    const side=this.side(which);if(side.pointer!==pointerId||side.pointer===null)return false;
    Object.assign(side,normalizedPad(x,y));return true;
  }
  end(which,pointerId){
    const side=this.side(which);if(side.pointer!==pointerId||side.pointer===null)return false;
    this.clear(which);return true;
  }
  clear(which){Object.assign(this.side(which),{pointer:null,x:0,y:0});}
  reset(){this.clear('left');this.clear('right');}
  sample(mode,keyboard={forward:0,left:0,turn:0,roll:0,pitch:0}){
    if(!['walk','body'].includes(mode))throw Error('Unknown control mode');
    const left=this.left.pointer!==null?this.left:null;
    const right=this.right.pointer!==null?this.right:null;
    const forward=left?left.y:(keyboard.forward||0);
    const lateral=left?-left.x:(keyboard.left||0);
    if(mode==='walk')return {forward,lateral,turn:right?-right.x:(keyboard.turn||0)};
    return {forward,lateral,roll:right?right.x:(keyboard.roll||0),pitch:right?right.y:(keyboard.pitch||0)};
  }
}
