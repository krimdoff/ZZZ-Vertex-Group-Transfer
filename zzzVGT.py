bl_info = {
    "name": "ZZZ Vertex Group Transfer",
    "author": "ReAgent74",
    "version": (1, 1, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > ZVGT",
    "description": "Transfer vertex group weights from a source mesh to a target mesh using nearest surface points. The mod was primarily developed for use in ZZZ",
    "category": "Object",
}

import bpy
from mathutils.bvhtree import BVHTree

# ---------------------------------------------------------------------------
# Локализация
# ---------------------------------------------------------------------------
TEXTS = {
    'RU': {
        'panel_label': "Перенос весов по ближайшей поверхности",
        'source_label': "Старая модель",
        'target_label': "Новая модель",
        'epsilon_label': "Порог веса",
        'skip_existing_label': "Пропускать существующие группы",
        'use_selection': "Взять выбранные объекты",
        'transfer': "Перенести Vertex Groups",
        'groups_on_target': "Групп на цели сейчас: {count}",
        'will_delete_all': "Все текущие группы цели будут удалены.",
        'will_skip_existing': "Существующие группы цели будут сохранены.",
        'report_info': "Источник: {source}; цель: {target}",
        'report_done': "Готово: {groups} групп; {assignments} назначений весов; вершин без соответствия: {unmapped}; пропущено групп: {skipped}; порядок: {order}",
        'report_source': "Источник: {source} ({source_verts} вершин)",
        'report_target': "Цель: {target} ({target_verts} вершин)",
        'report_order': "Порядок групп:",
        'order_ok': "OK",
        'order_error': "ОШИБКА",
        'error_active': "Сделай старую модель активной и выбери также новую.",
        'error_two_meshes': "Выбери два меша: старую и новую модели.",
        'error_source_target': "Укажи исходную и целевую модели.",
        'error_same': "Исходная и целевая модели должны отличаться.",
        'error_type': "Оба объекта должны быть типа MESH.",
        'error_empty': "Одна из моделей не содержит вершин.",
        'error_no_groups': "На исходной модели нет Vertex Groups.",
        'error_no_triangles': "У исходной модели нет треугольников.",
        'error_bvh': "Не удалось построить BVH исходной модели.",
    },
    'EN': {
        'panel_label': "Transfer weights by nearest surface",
        'source_label': "Source Mesh",
        'target_label': "Target Mesh",
        'epsilon_label': "Weight Threshold",
        'skip_existing_label': "Skip Existing Groups",
        'use_selection': "Use Selected Objects",
        'transfer': "Transfer Vertex Groups",
        'groups_on_target': "Groups on target now: {count}",
        'will_delete_all': "All current target groups will be deleted.",
        'will_skip_existing': "Existing target groups will be preserved.",
        'report_info': "Source: {source}; target: {target}",
        'report_done': "Done: {groups} groups; {assignments} weight assignments; unmapped vertices: {unmapped}; skipped groups: {skipped}; order: {order}",
        'report_source': "Source: {source} ({source_verts} vertices)",
        'report_target': "Target: {target} ({target_verts} vertices)",
        'report_order': "Group order:",
        'order_ok': "OK",
        'order_error': "ERROR",
        'error_active': "Make the old model active and also select the new one.",
        'error_two_meshes': "Select two meshes: old and new models.",
        'error_source_target': "Specify source and target meshes.",
        'error_same': "Source and target must be different.",
        'error_type': "Both objects must be of type MESH.",
        'error_empty': "One of the meshes has no vertices.",
        'error_no_groups': "Source mesh has no Vertex Groups.",
        'error_no_triangles': "Source mesh has no triangles.",
        'error_bvh': "Failed to build BVH for source mesh.",
    },
}

# ---------------------------------------------------------------------------
# Основная логика переноса
# ---------------------------------------------------------------------------
def transfer_weights(source, target, epsilon=0.00001, skip_existing=False, texts=None):
    if texts is None:
        texts = TEXTS['EN']

    if source is None or target is None:
        raise ValueError(texts['error_source_target'])
    if source == target:
        raise ValueError(texts['error_same'])
    if source.type != 'MESH' or target.type != 'MESH':
        raise TypeError(texts['error_type'])
    if len(source.data.vertices) == 0 or len(target.data.vertices) == 0:
        raise ValueError(texts['error_empty'])
    if len(source.vertex_groups) == 0:
        raise ValueError(texts['error_no_groups'])

    src_mesh = source.data
    dst_mesh = target.data
    src_mesh.calc_loop_triangles()
    triangles = [tuple(tri.vertices) for tri in src_mesh.loop_triangles]
    if not triangles:
        raise ValueError(texts['error_no_triangles'])

    # Мировые координаты, чтобы учитывать трансформации объектов.
    source_coords = [source.matrix_world @ v.co for v in src_mesh.vertices]
    bvh = BVHTree.FromPolygons(source_coords, triangles, all_triangles=True)
    if bvh is None:
        raise RuntimeError(texts['error_bvh'])

    source_groups = list(source.vertex_groups)
    source_weights = []
    for vertex in src_mesh.vertices:
        source_weights.append({
            assignment.group: assignment.weight
            for assignment in vertex.groups
        })

    # Определяем существующие группы цели, если нужно пропускать.
    existing_names = set()
    if skip_existing:
        existing_names = {g.name for g in target.vertex_groups}

    source_to_target = {}
    created_groups = []
    skipped_count = 0

    if skip_existing:
        # Сохраняем существующие группы, создаём только новые.
        for idx, src_group in enumerate(source_groups):
            if src_group.name in existing_names:
                source_to_target[idx] = None
                skipped_count += 1
            else:
                new_group = target.vertex_groups.new(name=src_group.name)
                source_to_target[idx] = new_group
                created_groups.append(new_group)
    else:
        # Прежнее поведение: удаляем все группы цели и создаём все группы источника.
        for group in list(target.vertex_groups):
            target.vertex_groups.remove(group)
        target_groups = [target.vertex_groups.new(name=g.name) for g in source_groups]
        for idx, tg in enumerate(target_groups):
            source_to_target[idx] = tg
        created_groups = target_groups

    assignments_count = 0
    unmapped_vertices = 0

    for vertex in dst_mesh.vertices:
        world_pos = target.matrix_world @ vertex.co
        result = bvh.find_nearest(world_pos)
        if result is None or result[2] is None:
            unmapped_vertices += 1
            continue

        nearest_point, normal, tri_index, distance = result
        tri = triangles[tri_index]
        a, b, c = (source_coords[i] for i in tri)
        v0, v1, v2 = b - a, c - a, nearest_point - a
        d00, d01, d11 = v0.dot(v0), v0.dot(v1), v1.dot(v1)
        d20, d21 = v2.dot(v0), v2.dot(v1)
        denominator = d00 * d11 - d01 * d01

        if abs(denominator) < 1e-20:
            nearest_id = min(
                tri,
                key=lambda i: (source_coords[i] - nearest_point).length_squared
            )
            bary = [0.0, 0.0, 0.0]
            bary[tri.index(nearest_id)] = 1.0
        else:
            w1 = (d11 * d20 - d01 * d21) / denominator
            w2 = (d00 * d21 - d01 * d20) / denominator
            w0 = 1.0 - w1 - w2
            bary = [
                max(0.0, min(1.0, w0)),
                max(0.0, min(1.0, w1)),
                max(0.0, min(1.0, w2)),
            ]
            total = sum(bary)
            if total > 0:
                bary = [w / total for w in bary]

        interpolated = {}
        for source_vertex_id, factor in zip(tri, bary):
            if factor <= 0:
                continue
            for group_id, weight in source_weights[source_vertex_id].items():
                interpolated[group_id] = interpolated.get(group_id, 0.0) + weight * factor

        for group_id, weight in interpolated.items():
            target_group = source_to_target.get(group_id)
            if target_group is not None and weight > epsilon:
                target_group.add([vertex.index], weight, 'REPLACE')
                assignments_count += 1

    # Проверка порядка групп.
    if skip_existing:
        expected_order = [g.name for g in source_groups if g.name not in existing_names]
        actual_order = [g.name for g in created_groups]
        order_ok = expected_order == actual_order
    else:
        order_ok = (
            len(source_groups) == len(created_groups)
            and all(source_groups[i].name == created_groups[i].name for i in range(len(source_groups)))
        )

    return {
        "groups": len(created_groups),
        "assignments": assignments_count,
        "unmapped": unmapped_vertices,
        "skipped": skipped_count,
        "order_ok": order_ok,
        "source_vertices": len(src_mesh.vertices),
        "target_vertices": len(dst_mesh.vertices),
    }

# ---------------------------------------------------------------------------
# Свойства сцены
# ---------------------------------------------------------------------------
class ZZZVGTransferProperties(bpy.types.PropertyGroup):
    source: bpy.props.PointerProperty(
        name="Source Mesh",
        description="Mesh from which Vertex Groups and weights are copied",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.type == 'MESH',
    )
    target: bpy.props.PointerProperty(
        name="Target Mesh",
        description="Mesh to which Vertex Groups and weights are transferred",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.type == 'MESH',
    )
    epsilon: bpy.props.FloatProperty(
        name="Weight Threshold",
        description="Weights below this value are not written",
        default=0.00001,
        min=0.0,
        max=0.1,
        precision=6,
    )
    skip_existing: bpy.props.BoolProperty(
        name="Skip Existing Groups",
        description="Do not transfer vertex groups whose names already exist on target mesh",
        default=False,
    )
    language: bpy.props.EnumProperty(
        name="Language",
        description="Addon interface language",
        items=[
            ('RU', 'Русский', 'Русский язык'),
            ('EN', 'English', 'English language'),
        ],
        default='RU',
    )

# ---------------------------------------------------------------------------
# Операторы
# ---------------------------------------------------------------------------
class ZZZVGTRANSFER_OT_use_selection(bpy.types.Operator):
    bl_idname = "zzz_vg_transfer.use_selection"
    bl_label = "Use Selected Objects"
    bl_description = "Assign active object as source and another selected mesh as target"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.zzz_vg_transfer
        texts = TEXTS[props.language]
        selected_meshes = [obj for obj in context.selected_objects if obj.type == 'MESH']
        active = context.view_layer.objects.active
        if active not in selected_meshes or active is None:
            self.report({'ERROR'}, texts['error_active'])
            return {'CANCELLED'}
        others = [obj for obj in selected_meshes if obj != active]
        if not others:
            self.report({'ERROR'}, texts['error_two_meshes'])
            return {'CANCELLED'}
        props.source = active
        props.target = others[0]
        self.report({'INFO'}, texts['report_info'].format(source=props.source.name, target=props.target.name))
        return {'FINISHED'}

class ZZZVGTRANSFER_OT_switch_language(bpy.types.Operator):
    bl_idname = "zzz_vg_transfer.switch_language"
    bl_label = "Switch Language"
    bl_description = "Switch addon interface language"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.zzz_vg_transfer
        props.language = 'EN' if props.language == 'RU' else 'RU'
        return {'FINISHED'}

class ZZZVGTRANSFER_OT_transfer(bpy.types.Operator):
    bl_idname = "zzz_vg_transfer.transfer"
    bl_label = "Transfer Vertex Groups"
    bl_description = "Remove target groups (unless skipping), create groups in source order and transfer weights by nearest surface"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.zzz_vg_transfer
        texts = TEXTS[props.language]
        try:
            result = transfer_weights(
                props.source,
                props.target,
                props.epsilon,
                props.skip_existing,
                texts
            )
        except Exception as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        message = texts['report_done'].format(
            groups=result['groups'],
            assignments=result['assignments'],
            unmapped=result['unmapped'],
            skipped=result['skipped'],
            order=texts['order_ok'] if result['order_ok'] else texts['order_error']
        )
        self.report({'INFO'}, message)
        print("\n[ZZZ Vertex Group Transfer]")
        print(message)
        print(texts['report_source'].format(source=props.source.name, source_verts=result['source_vertices']))
        print(texts['report_target'].format(target=props.target.name, target_verts=result['target_vertices']))
        print(texts['report_order'])
        for i, group in enumerate(props.target.vertex_groups):
            print(f"  {i}: {group.name}")
        return {'FINISHED'}

# ---------------------------------------------------------------------------
# Панель
# ---------------------------------------------------------------------------
class ZZZVGTRANSFER_PT_panel(bpy.types.Panel):
    bl_label = "ZZZ Vertex Group Transfer"
    bl_idname = "ZZZVGTRANSFER_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "ZZZ Tools"

    def draw(self, context):
        layout = self.layout
        props = context.scene.zzz_vg_transfer
        texts = TEXTS[props.language]

        # Кнопка смены языка
        row = layout.row()
        switch_text = "English" if props.language == 'RU' else "Русский"
        row.operator("zzz_vg_transfer.switch_language", text=switch_text, icon='WORLD')

        layout.label(text=texts['panel_label'], icon='MOD_DATA_TRANSFER')
        box = layout.box()
        box.prop(props, "source", text=texts['source_label'])
        box.prop(props, "target", text=texts['target_label'])
        box.prop(props, "epsilon", text=texts['epsilon_label'])
        box.prop(props, "skip_existing", text=texts['skip_existing_label'])

        layout.operator("zzz_vg_transfer.use_selection", text=texts['use_selection'], icon='EYEDROPPER')
        layout.separator()

        if props.target is not None and len(props.target.vertex_groups) > 0:
            layout.label(text=texts['groups_on_target'].format(count=len(props.target.vertex_groups)), icon='GROUP_VERTEX')
            if props.skip_existing:
                layout.label(text=texts['will_skip_existing'], icon='INFO')
            else:
                layout.label(text=texts['will_delete_all'], icon='INFO')

        row = layout.row()
        row.scale_y = 1.5
        row.operator("zzz_vg_transfer.transfer", text=texts['transfer'], icon='AUTOMERGE_ON')

# ---------------------------------------------------------------------------
# Регистрация
# ---------------------------------------------------------------------------
classes = (
    ZZZVGTransferProperties,
    ZZZVGTRANSFER_OT_use_selection,
    ZZZVGTRANSFER_OT_switch_language,
    ZZZVGTRANSFER_OT_transfer,
    ZZZVGTRANSFER_PT_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.zzz_vg_transfer = bpy.props.PointerProperty(type=ZZZVGTransferProperties)

def unregister():
    if hasattr(bpy.types.Scene, "zzz_vg_transfer"):
        del bpy.types.Scene.zzz_vg_transfer
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()