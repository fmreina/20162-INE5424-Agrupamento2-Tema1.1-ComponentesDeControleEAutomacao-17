// EPOS PC NIC Mediator Declarations

#ifndef __pc_nic_h
#define __pc_nic_h

#include <ethernet.h>
#include <system.h>
#include "machine.h"
#include "pcnet32.h"
#include "e100.h"
#include "c905.h"

__BEGIN_SYS

class PC_Ethernet: public Ethernet
{
    friend class PC;

private:
    typedef Traits<PC_Ethernet>::NICS NICS;
    typedef IF<NICS::Polymorphic, NIC_Base<Ethernet>, NICS::Get<0>::Result>::Result Device;
    static const unsigned int UNITS = NICS::Length;

public:
    typedef Data_Observer<Buffer, Protocol> Observer;
    typedef Data_Observed<Buffer, Protocol> Observed;

public:
    template<unsigned int UNIT = 0>
    PC_Ethernet(unsigned int u = UNIT) {
        _dev = reinterpret_cast<Device *>(NICS::Get<UNIT>::Result::get(u));
        db<PC_Ethernet>(TRC) << "NIC::NIC(u=" << UNIT << ",d=" << _dev << ") => " << this << endl;
    }
    ~PC_Ethernet() { _dev = 0; }

    int send(const Address & dst, const Protocol & prot, const void * data, unsigned int size) { return _dev->send(dst, prot, data, size); }
    int receive(Address * src, Protocol * prot, void * data, unsigned int size) { return _dev->receive(src, prot, data, size); }

    Buffer * alloc(const Address & dst, const Protocol & prot, unsigned int once, unsigned int always, unsigned int payload) { return _dev->alloc(this, dst, prot, once, always, payload); }
    int send(Buffer * buf) { return _dev->send(buf); }
    void free(Buffer * buf) { _dev->free(buf); }

    const Address & address() { return _dev->address(); }
    void address(const Address & address) { _dev->address(address); }

    const Statistics & statistics() { return _dev->statistics(); }

    void reset() { _dev->reset(); }

    void attach(Observer * obs, const Protocol & prot) { _dev->Ethernet::Observed::attach(obs, prot); }
    void detach(Observer * obs, const Protocol & prot) { _dev->Ethernet::Observed::detach(obs, prot); }
    void notify(const Protocol & prot, Buffer * buf) { _dev->Ethernet::Observed::notify(prot, buf); }

private:
    static void init();

private:
    Device * _dev;
};

__END_SYS

#endif
