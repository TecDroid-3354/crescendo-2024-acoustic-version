#include "limelight.hh"

#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>

#include "constants/strings.hh"

namespace td::ll
{

    std::shared_ptr<nt::NetworkTable> ll_table = nullptr;

    auto
    init() noexcept -> void
    {
        frc::ShuffleboardTab& lltab =frc::Shuffleboard::GetTab(k::str::drive_team_tab);

        ll_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        cs::HttpCamera camera { "Limelight Stream", "http://10.33.54.56:5801/stream.mjpg", cs::HttpCamera::HttpCameraKind::kMJPGStreamer };
        frc::CameraServer::StartAutomaticCapture(camera);

        lltab.Add(camera);

        lltab.AddBoolean("Has Target", []()
                         { return has_target(); });

        lltab.AddInteger("Target ID", []()
                         { return get_target_id(); });

        lltab.AddDouble("Angle X", []()
                        { return get_horizontal_angle_to_target().value(); });

        lltab.AddDouble("Angle Y", []()
                        { return get_vertical_angle_to_target().value(); });

        lltab.AddDouble("Distance", []()
                        { return get_distance_to_target().value(); });

        lltab.AddDouble("Target Type Height", []()
                        { return get_height_of_target_type().value(); });

        lltab.AddString("Target Type", []()
                        {
        switch (get_target_type()) {
        case target_type::AMP : return "Amp";
        case target_type::SPEAKER : return "Speaker";
        case target_type::STAGE : return "Stage";
        case target_type::SOURCE : return "Source";
        case target_type::NONE: return "None";
        }

        return "None"; });
    }

    [[nodiscard]] auto
    has_target() noexcept -> bool
    {
        return static_cast<int>(ll_table->GetNumber("tv", 0)) == 1;
    }

    [[nodiscard]] auto
    has_no_targets() noexcept -> bool
    {
        return !has_target();
    }

    [[nodiscard]] auto
    get_target_id() noexcept -> int
    {
        return static_cast<int>(ll_table->GetNumber("tid", -1.0)) > 0.0;
    }

    [[nodiscard]] auto
    get_horizontal_angle_to_target() noexcept -> units::degree_t
    {
        return units::degree_t{ll_table->GetNumber("tx", 0.0)};
    }

    [[nodiscard]] auto
    get_vertical_angle_to_target() noexcept -> units::degree_t
    {
        return units::degree_t{ll_table->GetNumber("ty", 0.0)};
    }

    [[nodiscard]] auto
    get_distance_to_target() noexcept -> units::meter_t
    {
        if (has_no_targets())
            return 0.0_m;

        units::meter_t target_height = get_height_of_target_type();

        units::radian_t angle_to_goal = mount_angle + get_vertical_angle_to_target();

        return (target_height - mount_height) / std::tan(angle_to_goal.value());
    }

    [[nodiscard]] auto
    get_target_type() noexcept -> target_type
    {
        if (!has_target())
            return target_type::NONE;

        switch (get_target_id())
        {
        case 1:
        case 2:
        case 9:
        case 10:
        {
            return target_type::SOURCE;
        }
        break;

        case 3:
        case 4:
        case 7:
        case 8:
        {
            return target_type::SPEAKER;
        }
        break;

        case 5:
        case 6:
        {
            return target_type::AMP;
        }
        break;

        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        {
            return target_type::STAGE;
        }
        break;
        }

        return target_type::NONE;
    }

    auto
    get_height_of_target_type() -> units::meter_t
    {
        if (has_no_targets())
            return 0.0_m;

        units::meter_t target_height = 0.0_m;

        switch (get_target_type())
        {
        case target_type::AMP:
            target_height = amp_height;
            break;
        case target_type::SPEAKER:
            target_height = speaker_height;
            break;
        case target_type::STAGE:
            target_height = stage_height;
            break;
        case target_type::SOURCE:
            target_height = source_height;
            break;
        case target_type::NONE:
            break;
        }

        return target_height;
    }

} // namespace td::ll
