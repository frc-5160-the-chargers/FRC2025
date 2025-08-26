package frc.chargers.hardware;



interface A {
    void x();
    void y();
    void z();
}

class B implements A {
    public void x() {}
    public void y() {}
    public void z() {}
}

class C extends B implements A {

}
