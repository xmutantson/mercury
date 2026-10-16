#include "datalink_layer/fifo_buffer.h"
#include <cstdio>
#include <cstring>
#include <new>
#include <initializer_list>

#ifdef FIFO_RESIZE_ALLOC_TEST
#include <cstdlib>
namespace {
void* allocations[512]; int live=0, released=0; bool reject=false;
void remember(void* p) { if(p) { for(auto& slot:allocations) if(!slot) {slot=p;++live;return;} std::abort(); } }
void forget(void* p) { if(p) for(auto& slot:allocations) if(slot==p) {slot=nullptr;--live;++released;return;} }
}
extern "C" void* __real__Znam(std::size_t);
extern "C" void* __real__ZnamRKSt9nothrow_t(std::size_t,const std::nothrow_t&);
extern "C" void __real__ZdaPv(void*);
extern "C" void* __wrap__Znam(std::size_t n) {if(reject) throw std::bad_alloc();auto p=__real__Znam(n);remember(p);return p;}
extern "C" void* __wrap__ZnamRKSt9nothrow_t(std::size_t n,const std::nothrow_t& t) {if(reject)return nullptr;auto p=__real__ZnamRKSt9nothrow_t(n,t);remember(p);return p;}
extern "C" void __wrap__ZdaPv(void* p) {forget(p);__real__ZdaPv(p);}
#endif
namespace {
int checks=0, failures=0;
void check(bool ok,const char* name) {++checks;if(!ok)++failures;std::printf("FIFO_RESIZE check=%s ok=%d\n",name,ok);}
int resize(cl_fifo_buffer& q,int size) {try {return q.set_size(size);}catch(const std::bad_alloc&) {return -2;}}
void seed(cl_fifo_buffer& q,bool wrapped) {
    char bytes[12];for(int i=0;i<12;++i)bytes[i]=(char)(i*37+11);
    check(q.set_size(16)==SUCCESSFUL,"initial_size");
    check(q.push(bytes,12)==12,"initial_push");char out[8];
    check(q.pop(out,8)==8 && !std::memcmp(out,bytes,8),"initial_pop_key");
    if(wrapped)check(q.push(bytes,8)==8,"wrap_push");
}
void fresh(cl_fifo_buffer& q,int size) {
    char key[64],out[64];for(int i=0;i<64;++i)key[i]=(char)(i*73+19);
    check(q.push(key,size-1)==size-1,"fresh_push");
    check(q.push(key,1)==0,"full_reject");
    check(q.pop(out,size)==size-1 && !std::memcmp(key,out,size-1),"fresh_key_exact");
    check(q.push_front(key,size-1)==size-1,"front_push");
    check(q.pop(out,size)==size-1 && !std::memcmp(key,out,size-1),"front_key_exact");
    q.push(key,1);q.flush();check(q.get_free_size()==size-1 && q.pop(out,1)==0,"flush_empty");
}
}
int run_fifo_resize_tests() {
    checks=failures=0;
    for(bool wrapped:{false,true}) for(int size:{4,16,32}) {
#ifdef FIFO_RESIZE_ALLOC_TEST
        int before=live;
#endif
        {
            cl_fifo_buffer q;seed(q,wrapped);
#ifdef FIFO_RESIZE_ALLOC_TEST
            int freed=released;
#endif
            check(resize(q,size)==SUCCESSFUL,"resize_success");
            bool empty=q.get_size()==size-1 && q.get_free_size()==size-1;
            check(empty,"occupied_resize_empty");
#ifdef FIFO_RESIZE_ALLOC_TEST
            check(live==before+1 && released==freed+1,"replacement_releases_old");
#endif
            // A failed invariant must not make the baseline proof itself access stale cursors.
            // Short-circuit the read on a broken geometry; always count the assertion.
            char out;check(empty && q.pop(&out,1)==0,"no_phantom_byte");
            // Public free capacity cannot prove cursor bounds after shrink. Reset
            // before exercising fresh operations, even when it appears empty.
            q.flush();fresh(q,size);
        }
#ifdef FIFO_RESIZE_ALLOC_TEST
        check(live==before,"destruction_balance");
#endif
    }
    for(int invalid:{-1,0,1}) {
        cl_fifo_buffer q;char key[4]={0,37,(char)193,7},out[4];
        check(q.set_size(16)==SUCCESSFUL && q.push(key,4)==4,"invalid_setup");
        int rc=resize(q,invalid);check(rc==ERROR_,"invalid_return");
        bool intact=q.get_size()==15 && q.get_free_size()==11;
        check(intact,"invalid_keeps_geometry");
        check(intact && q.pop(out,4)==4 && !std::memcmp(key,out,4),"invalid_keeps_key");
    }
#ifdef FIFO_RESIZE_ALLOC_TEST
    for(bool wrapped:{false,true}) {
        cl_fifo_buffer q;seed(q,wrapped);int oldlive=live,oldfree=q.get_free_size();
        reject=true;int rc=resize(q,32);reject=false;
        check(rc==ERROR_,"allocation_error_return");
        check(live==oldlive && q.get_size()==15 && q.get_free_size()==oldfree,"allocation_preserves_geometry");
        char out[16],want[12];for(int i=0;i<4;++i)want[i]=(char)((i+8)*37+11);
        if(wrapped)for(int i=0;i<8;++i)want[i+4]=(char)(i*37+11);
        int n=wrapped?12:4;check(q.pop(out,16)==n && !std::memcmp(out,want,n),"allocation_preserves_key");
        check(resize(q,8)==SUCCESSFUL && q.get_free_size()==7,"allocation_failure_recovery");q.flush();fresh(q,8);
    }
    check(live==0,"total_allocation_balance");
    // Reclaim only baseline leaks after all lifecycle/allocation assertions.
    for(auto& p:allocations)if(p){__real__ZdaPv(p);p=nullptr;}live=0;
#endif
    for(int repeat=0;repeat<4;++repeat) {cl_fifo_buffer q;check(q.set_size(2)==SUCCESSFUL,"minimum_ring");fresh(q,2);}
    std::printf("FIFO_RESIZE_RESULT checks=%d failures=%d\n",checks,failures);return failures;
}
