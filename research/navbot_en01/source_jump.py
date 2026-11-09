"""Pinned firmware jump commands in iteration time; physical offsets stay explicit."""
from __future__ import annotations
import hashlib, json, math, shutil, subprocess, tempfile
from pathlib import Path
from .servo_mapping import normal_positions, jump_positions, logical_angle_since_squat

UPSTREAM='3139f2fb32b38d10611feb905e16f8aa012f2f2e'
FIRMWARE_BLOB='c5c69e515d7d2a22edc3779667fc3edfe4045ae9'
SERVO_BLOB='cb945e94855da31e050b9b482a9ad2682376811e'
WEB_BLOB='0400a92d7f1de9d4374367c5c3936d39032a5b5f'
PERIODS_MS=(1,2,5,10)
ANCHORS=(-.04,0.,.04)

class JumpCommands:
    """Port of jump_loop; normal leg-loop resumption is represented separately.

    The source calls lqr_balance_loop BEFORE leg_loop, so wheels see flag_before.
    A zero Speed/ACC byte is retained as protocol data, not converted to radians/s.
    """
    def __init__(self,height=38):
        self.height=height; self.flag=0; self.positions=normal_positions(height); self.iteration=0

    def tick(self,*,previous_stop=False,jump=False):
        before=self.flag; writes=[]
        if previous_stop and jump and self.flag==0:
            self.positions=jump_positions('initial'); writes.append('extend'); self.flag=1
        if self.flag>0:
            self.flag+=1
            if 30<self.flag<35:
                self.positions=jump_positions('return'); writes.append('return'); self.flag=40
            if self.flag>200:self.flag=0; writes.append('resume')
        if self.flag==0:self.positions=normal_positions(self.height)
        out={'iteration':self.iteration,'flag_before':before,'flag_after':self.flag,
             'positions':list(self.positions),'writes':writes,'source_jump_gate':before!=0}
        self.iteration+=1
        return out

def cad_targets(positions,*,anchor_q_rad=None,height=38):
    """Declared CAD q at source height=38; no claimed mechanical-zero calibration."""
    if anchor_q_rad is None or not math.isfinite(anchor_q_rad):
        raise ValueError('An explicit uncalibrated CAD anchor is required')
    normal=normal_positions(height)
    out=[anchor_q_rad+logical_angle_since_squat(p,i+1)-logical_angle_since_squat(normal[i],i+1)
         for i,p in enumerate(positions)]
    if any(not -.22<=v<=.65 for v in out):raise ValueError('Source command outside declared diagnostic joint limits')
    return out

def _source_function(text,name):
    start=text.index('void '+name+'('); brace=text.index('{',start); depth=0
    for i in range(brace,len(text)):
        depth+=(text[i]=='{')-(text[i]=='}')
        if depth==0:return text[start:i+1]
    raise ValueError('Incomplete firmware function')

def audit_source(firmware,servo,web):
    def read(path,want):
        b=Path(path).read_bytes(); actual=hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()
        if actual!=want:raise ValueError('Pinned upstream Git blob mismatch')
        return b.decode()
    text=read(firmware,FIRMWARE_BLOB); servo_text=read(servo,SERVO_BLOB); web_text=read(web,WEB_BLOB)
    loop=_source_function(text,'loop'); jump=_source_function(text,'jump_loop')
    if not loop.index('lqr_balance_loop();')<loop.index('leg_loop();'):raise ValueError('Unexpected controller order')
    if 'g_h=38' not in web_text.replace(' ','') or 'id="jump"' not in web_text:raise ValueError('Unknown web command source')
    if 'offbuf[i * 7] = ACC[i]' not in servo_text or 'V = Speed[i]' not in servo_text:raise ValueError('Unknown servo protocol writer')
    compiler=shutil.which('g++')
    if compiler is None:raise RuntimeError('Source-state-machine verification requires g++')
    # Compile the exact pinned jump_loop body. Stubs only supply its scalar inputs
    # and capture actual SyncWritePosEx commands, with the original C++ s16 casts.
    prefix='''#include <iostream>
#include <cstdint>
using s16=int16_t; using u16=uint16_t; using byte=uint8_t;
enum {STOP,JUMP}; struct {int dir_last,dir;} wrobot;
int jump_flag=0; byte ID[2]={1,2},ACC[2];u16 Speed[2];s16 Position[2];
struct {void SyncWritePosEx(byte*,int,s16*p,u16*s,byte*a){std::cout<<"W "<<p[0]<<" "<<p[1]<<" "<<s[0]<<" "<<int(a[0])<<"\\n";}} sms_sts;
'''
    tail='''\nint main(){for(int i=0;i<480;i++){wrobot.dir_last=(i==0||i==250)?STOP:JUMP;wrobot.dir=JUMP;std::cout<<"I "<<i<<"\\n";jump_loop();std::cout<<"F "<<jump_flag<<"\\n";}}\n'''
    with tempfile.TemporaryDirectory() as td:
        src=Path(td)/'probe.cpp'; binary=Path(td)/'probe';src.write_text(prefix+jump+tail)
        subprocess.run([compiler,'-std=c++17','-O2',str(src),'-o',str(binary)],check=True,capture_output=True)
        actual=subprocess.check_output([str(binary)],text=True).splitlines()
    expected=[]; fsm=JumpCommands(); transitions=[]
    for i in range(480):
        out=fsm.tick(previous_stop=i in (0,250),jump=True);expected.append('I '+str(i))
        for name in out['writes']:
            if name in ('extend','return'):
                pos=jump_positions('initial' if name=='extend' else 'return')
                expected.append(f'W {pos[0]} {pos[1]} 0 0')
            transitions.append({'iteration':i,'event':name})
        expected.append('F '+str(out['flag_after']))
    if actual!=expected:raise ValueError('Python command machine disagrees with compiled original firmware')
    return {'schema':'en01-source-jump-audit/v1','upstream_commit':UPSTREAM,
            'firmware_blob':FIRMWARE_BLOB,'servo_blob':SERVO_BLOB,'web_blob':WEB_BLOB,
            'compiled_original_cpp_iterations':480,'all_commands_and_flags_match':True,
            'transitions':transitions,'normal_positions_38':list(normal_positions(38)),
            'extend_positions':list(jump_positions('initial')),'return_positions':list(jump_positions('return')),
            'launch_targets_at_anchor_zero_rad':cad_targets(jump_positions('initial'),anchor_q_rad=0),
            'return_targets_at_anchor_zero_rad':cad_targets(jump_positions('return'),anchor_q_rad=0),
            'source_loop_fixed_period':False,'source_voltage_to_torque_calibrated':False,
            'speed_acc_zero_interpretation':'Exact zero protocol bytes; physical internal-servo response unverified'}
