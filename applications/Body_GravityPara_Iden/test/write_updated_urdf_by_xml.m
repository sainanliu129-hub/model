function write_updated_urdf_by_xml(source_file, out_file, link_params)
% 兼容旧版 MATLAB：通过 XML 直接更新 <link><inertial> 的 mass/origin/inertia
doc = xmlread(source_file);
links = doc.getElementsByTagName('link');

for i = 1:numel(link_params)
    target_name = link_params(i).name;
    link_elem = [];
    for k = 0:links.getLength()-1
        node = links.item(k);
        if node.hasAttributes()
            attr = node.getAttributes().getNamedItem('name');
            if ~isempty(attr) && strcmp(char(attr.getNodeValue()), target_name)
                link_elem = node;
                break;
            end
        end
    end
    if isempty(link_elem)
        error('XML 中未找到 link: %s', target_name);
    end

    inertial = get_or_create_child(link_elem, 'inertial', doc);
    mass_node = get_or_create_child(inertial, 'mass', doc);
    origin_node = get_or_create_child(inertial, 'origin', doc);
    inertia_node = get_or_create_child(inertial, 'inertia', doc);

    mass_node.setAttribute('value', sprintf('%.16g', link_params(i).mass));
    origin_node.setAttribute('xyz', sprintf('%.16g %.16g %.16g', ...
        link_params(i).com(1), link_params(i).com(2), link_params(i).com(3)));
    if isempty(char(origin_node.getAttribute('rpy')))
        origin_node.setAttribute('rpy', '0 0 0');
    end

    inertia_node.setAttribute('ixx', sprintf('%.16g', link_params(i).ixx));
    inertia_node.setAttribute('ixy', sprintf('%.16g', link_params(i).ixy));
    inertia_node.setAttribute('ixz', sprintf('%.16g', link_params(i).ixz));
    inertia_node.setAttribute('iyy', sprintf('%.16g', link_params(i).iyy));
    inertia_node.setAttribute('iyz', sprintf('%.16g', link_params(i).iyz));
    inertia_node.setAttribute('izz', sprintf('%.16g', link_params(i).izz));
end

xmlwrite(out_file, doc);
end

function child = get_or_create_child(parent, name, doc)
child = [];
children = parent.getChildNodes();
for i = 0:children.getLength()-1
    n = children.item(i);
    if n.getNodeType() == n.ELEMENT_NODE && strcmp(char(n.getNodeName()), name)
        child = n;
        return;
    end
end
child = doc.createElement(name);
parent.appendChild(child);
end
