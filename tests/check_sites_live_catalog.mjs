import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';

const source=fs.readFileSync('hosting/github-live-sw.js','utf8');
const path=process.argv[2];
assert.ok(path,'Expected downloaded real GitHub library.json');
const catalog=JSON.parse(fs.readFileSync(path,'utf8'));
const listeners={};
const ctx=vm.createContext({
 self:{location:{origin:'https://private-chatgpt-site.example'},skipWaiting:()=>Promise.resolve(),
       clients:{claim:()=>Promise.resolve()},addEventListener:(type,cb)=>{listeners[type]=cb;}},
 caches:{},fetch(){},crypto:{},URL,Date,Number,Map,Set,console
});
vm.runInContext(source,ctx);
const validated=ctx.validateCatalog(catalog);
assert.equal(catalog.schema,'hexapod-library/v1');
assert.ok(catalog.revision>=36,'Expected working library at least revision 36');
assert.ok(catalog.entries.length>=57,'Expected 57+ recordings');
assert.ok(validated.files.size>=198,'Expected 198+ immutable assets');
const microduck=catalog.entries.find(x=>x.id==='microduck-standup-v1');
assert.ok(microduck,'MicroDuck is absent');
assert.equal(microduck.files.length,10);
assert.equal(microduck.href,'/hexapod/microduck-standup-v1/');
assert.ok(microduck.files.some(x=>x.path.endsWith('/index.html')));
console.log('PASS: live working library',catalog.revision,'entries',catalog.entries.length,'assets',validated.files.size,'MicroDuck files',microduck.files.length);
