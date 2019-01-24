use crate::{Model, DetectionMsg};
use nannou::ui::prelude::*;

pub(crate) fn update(model: &mut Model) {
    let ui = &mut model.ui.set_widgets();

    widget::Canvas::new().rgb(0.0, 0.0, 0.2).set(model.ids.background, ui);

    fn slider(val: f32, min: f32, max: f32) -> widget::Slider<'static, f32> {
        widget::Slider::new(val, min, max)
            .w_h(200.0, 30.0)
            .label_font_size(15)
            .rgb(0.3, 0.3, 0.3)
            .label_rgb(1.0, 1.0, 1.0)
            .border(0.0)
    }

    for value in slider(model.controls.threshold, 0.0, 2.0)
        .top_left_with_margin(20.0)
        .label(&format!("Threshold: {}", model.controls.threshold))
        .set(model.ids.threshold, ui)
    {
        model.controls.threshold = value;
        model.rx.detection.send(DetectionMsg::Threshold(value)).ok();
    }
    
    for value in slider(model.controls.curve as f32, 0.0, 10.0)
        .down(10.0)
        .label(&format!("Curve: {}", model.controls.curve))
        .set(model.ids.curve, ui)
    {
        model.controls.curve = value as i32;
        model.rx.detection.send(DetectionMsg::Curve(value as i32)).ok();
    }
    
    for value in slider(model.controls.joint_cutoff as f32, 0.0, 0.2)
        .down(10.0)
        .label(&format!("Joint Cutoff: {}", model.controls.joint_cutoff))
        .set(model.ids.joint_cutoff, ui)
    {
        model.controls.joint_cutoff = value;
        model.rx.detection.send(DetectionMsg::JointCutOff(value)).ok();
    }
}
