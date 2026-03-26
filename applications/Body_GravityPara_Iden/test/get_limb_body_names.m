function names = get_limb_body_names(limb_name)
switch limb_name
    case 'left_leg'
        names = {'leg_l1_link','leg_l2_link','leg_l3_link','leg_l4_link','leg_l5_link','leg_l6_link'};
    case 'right_leg'
        names = {'leg_r1_link','leg_r2_link','leg_r3_link','leg_r4_link','leg_r5_link','leg_r6_link'};
    case 'left_arm'
        names = {'arm_l1_link','arm_l2_link','arm_l3_link'};
    case 'right_arm'
        names = {'arm_r1_link','arm_r2_link','arm_r3_link'};
    otherwise
        error('未知 limb: %s', limb_name);
end
end
