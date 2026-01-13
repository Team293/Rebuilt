package frc.lib.subsystem;

public interface BaseIO<I extends BaseInputClass> {
    void updateInputs(I inputs);
}