/*
 * SettingsJPanel.java
 *
 * Created on 25.7.2010, 22:34:22
 */
package LineFollowerSimulator;

/**
 * GUI for adjusting robot properities
 * 
 * @author Ondrej Stanek
 */
public class SettingsJPanel extends javax.swing.JPanel {

    private Robot robot;
    private final double COEF = 128.0;

    /** Creates new form SettingsJPanel
     * @param robot the robot that will be adjusted with this panel
     */
    public SettingsJPanel(Robot robot) {
	initComponents();
	this.robot = robot;
    }

    /**
     *
     */
    public void UpdateAllSettings(){
	sbWidthAdjustmentValueChanged(null);
	sbPositionAdjustmentValueChanged(null);
	sbGaugeAdjustmentValueChanged(null);
	sbProportionalAdjustmentValueChanged(null);
	sbIntegralAdjustmentValueChanged(null);
	sbDerivateAdjustmentValueChanged(null);
	sbSpeedAdjustmentValueChanged(null);
	sbAccelerationAdjustmentValueChanged(null);
	jSpinnerFrequencyStateChanged(null);
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jSlider2 = new javax.swing.JSlider();
        jPanel1 = new javax.swing.JPanel();
        sbGauge = new javax.swing.JScrollBar();
        sbPosition = new javax.swing.JScrollBar();
        sbWidth = new javax.swing.JScrollBar();
        jLabel1 = new javax.swing.JLabel();
        jLabel2 = new javax.swing.JLabel();
        jLabel3 = new javax.swing.JLabel();
        jPanel2 = new javax.swing.JPanel();
        sbProportional = new javax.swing.JScrollBar();
        sbIntegral = new javax.swing.JScrollBar();
        sbDerivate = new javax.swing.JScrollBar();
        jLabel4 = new javax.swing.JLabel();
        jLabel5 = new javax.swing.JLabel();
        jLabel6 = new javax.swing.JLabel();
        jLabel7 = new javax.swing.JLabel();
        jSpinnerFrequency = new javax.swing.JSpinner();
        jLabel8 = new javax.swing.JLabel();
        jLabel9 = new javax.swing.JLabel();
        sbSpeed = new javax.swing.JScrollBar();
        jPanel3 = new javax.swing.JPanel();
        jLabel10 = new javax.swing.JLabel();
        sbAcceleration = new javax.swing.JScrollBar();
        jButton1 = new javax.swing.JButton();

        jPanel1.setBorder(javax.swing.BorderFactory.createTitledBorder("Geometry"));

        sbGauge.setMinimum(5);
        sbGauge.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbGauge.setValue(20);
        sbGauge.setName(""); // NOI18N
        sbGauge.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbGaugeAdjustmentValueChanged(evt);
            }
        });

        sbPosition.setMinimum(1);
        sbPosition.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbPosition.setValue(20);
        sbPosition.setName(""); // NOI18N
        sbPosition.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbPositionAdjustmentValueChanged(evt);
            }
        });

        sbWidth.setMinimum(10);
        sbWidth.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbWidth.setValue(20);
        sbWidth.setName(""); // NOI18N
        sbWidth.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbWidthAdjustmentValueChanged(evt);
            }
        });

        jLabel1.setText("Wheel gauge");

        jLabel2.setText("Sensors position");

        jLabel3.setText("Sensor module width");

        javax.swing.GroupLayout jPanel1Layout = new javax.swing.GroupLayout(jPanel1);
        jPanel1.setLayout(jPanel1Layout);
        jPanel1Layout.setHorizontalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel1Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(jLabel1)
                    .addComponent(jLabel2)
                    .addComponent(jLabel3))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(sbPosition, javax.swing.GroupLayout.DEFAULT_SIZE, 139, Short.MAX_VALUE)
                    .addComponent(sbWidth, javax.swing.GroupLayout.DEFAULT_SIZE, 139, Short.MAX_VALUE)
                    .addComponent(sbGauge, javax.swing.GroupLayout.DEFAULT_SIZE, 139, Short.MAX_VALUE))
                .addContainerGap())
        );
        jPanel1Layout.setVerticalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel1Layout.createSequentialGroup()
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(jLabel1)
                    .addComponent(sbGauge, javax.swing.GroupLayout.PREFERRED_SIZE, 16, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(sbPosition, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel2))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel3)
                    .addComponent(sbWidth, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
        );

        jPanel2.setBorder(javax.swing.BorderFactory.createTitledBorder("PID regulator"));

        sbProportional.setMaximum(256);
        sbProportional.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbProportional.setValue(50);
        sbProportional.setName(""); // NOI18N
        sbProportional.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbProportionalAdjustmentValueChanged(evt);
            }
        });

        sbIntegral.setMaximum(256);
        sbIntegral.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbIntegral.setName(""); // NOI18N
        sbIntegral.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbIntegralAdjustmentValueChanged(evt);
            }
        });

        sbDerivate.setMaximum(1024);
        sbDerivate.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbDerivate.setName(""); // NOI18N
        sbDerivate.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbDerivateAdjustmentValueChanged(evt);
            }
        });

        jLabel4.setText("Proportional (P)");

        jLabel5.setText("Integral (I)");

        jLabel6.setText("Derivative (D)");

        jLabel7.setText("Update frequency");

        jSpinnerFrequency.setModel(new javax.swing.SpinnerNumberModel(30, 1, 50, 1));
        jSpinnerFrequency.setInheritsPopupMenu(true);
        jSpinnerFrequency.setValue(30);
        jSpinnerFrequency.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                jSpinnerFrequencyStateChanged(evt);
            }
        });

        jLabel8.setText("Hz");

        jLabel9.setText("Speed");

        sbSpeed.setMaximum(1024);
        sbSpeed.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbSpeed.setToolTipText("");
        sbSpeed.setValue(200);
        sbSpeed.setName(""); // NOI18N
        sbSpeed.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbSpeedAdjustmentValueChanged(evt);
            }
        });

        javax.swing.GroupLayout jPanel2Layout = new javax.swing.GroupLayout(jPanel2);
        jPanel2.setLayout(jPanel2Layout);
        jPanel2Layout.setHorizontalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel2Layout.createSequentialGroup()
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addContainerGap()
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addComponent(jLabel5)
                            .addComponent(jLabel6)
                            .addComponent(jLabel4)
                            .addComponent(jLabel9))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(sbSpeed, javax.swing.GroupLayout.DEFAULT_SIZE, 152, Short.MAX_VALUE)
                            .addComponent(sbIntegral, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 152, Short.MAX_VALUE)
                            .addComponent(sbDerivate, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 152, Short.MAX_VALUE)
                            .addComponent(sbProportional, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 152, Short.MAX_VALUE))
                        .addGap(21, 21, 21))
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addGap(31, 31, 31)
                        .addComponent(jLabel7)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(jSpinnerFrequency, javax.swing.GroupLayout.PREFERRED_SIZE, 45, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(jLabel8, javax.swing.GroupLayout.PREFERRED_SIZE, 20, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap())
        );
        jPanel2Layout.setVerticalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel2Layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel4)
                    .addComponent(sbProportional, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(sbIntegral, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel5))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel6)
                    .addComponent(sbDerivate, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(sbSpeed, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel9))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(jLabel7)
                    .addComponent(jSpinnerFrequency, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel8))
                .addGap(5, 5, 5))
        );

        jPanel3.setBorder(javax.swing.BorderFactory.createTitledBorder("Physics"));

        jLabel10.setText("Acceleration");

        sbAcceleration.setMaximum(256);
        sbAcceleration.setOrientation(javax.swing.JScrollBar.HORIZONTAL);
        sbAcceleration.setValue(20);
        sbAcceleration.setName(""); // NOI18N
        sbAcceleration.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                sbAccelerationAdjustmentValueChanged(evt);
            }
        });

        javax.swing.GroupLayout jPanel3Layout = new javax.swing.GroupLayout(jPanel3);
        jPanel3.setLayout(jPanel3Layout);
        jPanel3Layout.setHorizontalGroup(
            jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel3Layout.createSequentialGroup()
                .addGap(30, 30, 30)
                .addComponent(jLabel10)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(sbAcceleration, javax.swing.GroupLayout.DEFAULT_SIZE, 153, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel3Layout.setVerticalGroup(
            jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel3Layout.createSequentialGroup()
                .addGroup(jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(jLabel10)
                    .addComponent(sbAcceleration, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        jButton1.setText("Reset Position");
        jButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton1ActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(jPanel1, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
            .addComponent(jPanel2, 0, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
            .addComponent(jPanel3, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addContainerGap(92, Short.MAX_VALUE)
                .addComponent(jButton1)
                .addGap(79, 79, 79))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addComponent(jPanel1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jPanel2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jPanel3, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(jButton1)
                .addContainerGap(15, Short.MAX_VALUE))
        );
    }// </editor-fold>//GEN-END:initComponents


    private void sbWidthAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbWidthAdjustmentValueChanged
	robot.lineSensor.setGeomtery(sbWidth.getValue(), sbPosition.getValue());
	sbWidth.setToolTipText(Integer.toString(sbWidth.getValue()));
}//GEN-LAST:event_sbWidthAdjustmentValueChanged

    private void sbPositionAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbPositionAdjustmentValueChanged
	robot.lineSensor.setGeomtery(sbWidth.getValue(), sbPosition.getValue());
	sbPosition.setToolTipText(Integer.toString(sbPosition.getValue()));
}//GEN-LAST:event_sbPositionAdjustmentValueChanged

    private void sbGaugeAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbGaugeAdjustmentValueChanged
	robot.setWheelGauge(sbGauge.getValue());
	sbGauge.setToolTipText(Integer.toString(sbGauge.getValue()));
}//GEN-LAST:event_sbGaugeAdjustmentValueChanged

    private void sbProportionalAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbProportionalAdjustmentValueChanged
	double p = sbProportional.getValue() / COEF;
	robot.regulator.setP(p);
	sbProportional.setToolTipText(Double.toString(p));
    }//GEN-LAST:event_sbProportionalAdjustmentValueChanged

    private void sbIntegralAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbIntegralAdjustmentValueChanged
	double i = sbIntegral.getValue() / COEF;
	robot.regulator.setI(i);
	sbIntegral.setToolTipText(Double.toString(i));
    }//GEN-LAST:event_sbIntegralAdjustmentValueChanged

    private void sbDerivateAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbDerivateAdjustmentValueChanged
	double d = sbDerivate.getValue() / COEF;
	robot.regulator.setD(d);
	sbDerivate.setToolTipText(Double.toString(d));
    }//GEN-LAST:event_sbDerivateAdjustmentValueChanged

    private void sbSpeedAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbSpeedAdjustmentValueChanged
	double speed = sbSpeed.getValue() / COEF;
	robot.regulator.setSpeed(speed);
	sbSpeed.setToolTipText(Double.toString(speed));
    }//GEN-LAST:event_sbSpeedAdjustmentValueChanged

    private void sbAccelerationAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_sbAccelerationAdjustmentValueChanged
	double a = sbAcceleration.getValue() / COEF;
	robot.motorController.setAcceleration(a);
	sbAcceleration.setToolTipText(Double.toString(a));
    }//GEN-LAST:event_sbAccelerationAdjustmentValueChanged

    private void jSpinnerFrequencyStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_jSpinnerFrequencyStateChanged
	int value = ((Integer) jSpinnerFrequency.getValue());
	robot.regulator.SetFrequency(value);
    }//GEN-LAST:event_jSpinnerFrequencyStateChanged

    private void jButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
	robot.ResetPosition();
    }//GEN-LAST:event_jButton1ActionPerformed
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton jButton1;
    private javax.swing.JLabel jLabel1;
    private javax.swing.JLabel jLabel10;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel8;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JSlider jSlider2;
    private javax.swing.JSpinner jSpinnerFrequency;
    private javax.swing.JScrollBar sbAcceleration;
    private javax.swing.JScrollBar sbDerivate;
    private javax.swing.JScrollBar sbGauge;
    private javax.swing.JScrollBar sbIntegral;
    private javax.swing.JScrollBar sbPosition;
    private javax.swing.JScrollBar sbProportional;
    private javax.swing.JScrollBar sbSpeed;
    private javax.swing.JScrollBar sbWidth;
    // End of variables declaration//GEN-END:variables
}
