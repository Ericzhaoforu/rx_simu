#!/usr/bin/env python3
"""Procedural 2026 RoboBuoy geometry, in metres. No legacy VRX buoy assets."""
import math
from pathlib import Path
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'models/robotx_2026_robobuoy'


def add(p, tag, text=None, **a):
    n = E.SubElement(p, tag, a)
    if text is not None: n.text = str(text)
    return n


def mesh(name, vertices, faces):
    # STL has no embedded materials; SDF owns surface colours unambiguously.
    lines=['solid '+name]
    for face in faces:
        a,b,c=[vertices[i] for i in face]
        u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
        normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        norm=math.sqrt(sum(x*x for x in normal));normal=[x/norm for x in normal]
        lines+=['facet normal '+' '.join(map(str,normal)),'outer loop']
        lines+=['vertex '+' '.join(map(str,p)) for p in (a,b,c)]
        lines+=['endloop','endfacet']
    lines+=['endsolid '+name]
    (OUT/'meshes'/f'{name}.stl').write_text('\n'.join(lines)+'\n')


def ring(name, inner, outer, bottom, top, segments=64):
    verts=[]; faces=[]
    for z in (bottom, top):
        for r in (inner, outer):
            verts += [[r*math.cos(i*2*math.pi/segments), r*math.sin(i*2*math.pi/segments), z] for i in range(segments)]
    for i in range(segments):
        j=(i+1)%segments
        for a,b,c,d in [(i,j,segments+j,segments+i),
                        (2*segments+i,3*segments+i,3*segments+j,2*segments+j),
                        (segments+i,segments+j,3*segments+j,3*segments+i),
                        (i,2*segments+i,2*segments+j,j)]:
            faces.extend([(a,b,c),(a,c,d)])
    mesh(name,verts,faces)


def main():
    (OUT/'meshes').mkdir(parents=True,exist_ok=True)
    panels=[]; diamonds=[]
    for k in range(4):
        def point(x,z,offset=0):
            f=(z-.10)/.3048; y=-(.2159+(.092075-.2159)*f)-offset
            a=k*math.pi/2
            return [x*math.cos(a)-y*math.sin(a), x*math.sin(a)+y*math.cos(a), z]
        panels += [point(-.2159,.10),point(.2159,.10),point(.092075,.4048),point(-.092075,.4048)]
        # Official mark: approx 8 inches high, bottom 2.5 inches above panel base.
        diamonds += [point(0,.1635,.001),point(.065,.2651,.001),point(0,.3667,.001),point(-.065,.2651,.001)]
    faces=[f for k in range(4) for f in [(4*k,4*k+1,4*k+2),(4*k,4*k+2,4*k+3),
                                       (4*k+2,4*k+1,4*k),(4*k+3,4*k+2,4*k)]]
    mesh('panels',panels,faces);mesh('diamonds',diamonds,faces)
    ring('beacon_pipe',.040,.04445,.435,.56835)
    ring('sunhat',.045,.127,.56835,.57310)
    root=E.Element('sdf',version='1.9');m=add(root,'model',name='robotx_2026_robobuoy')
    add(m,'static','true');link=add(m,'link',name='body')
    def part(name,geometry,pose='0 0 0 0 0 0',color='.95 .95 .95 1',collision=False):
        n=add(link,'visual',name=name);add(n,'pose',pose);geom=add(n,'geometry')
        tag,values=geometry
        node=add(geom,tag)
        for key,val in values.items():add(node,key,val)
        mat=add(n,'material');add(mat,'ambient',color);add(mat,'diffuse',color)
        if name.startswith('light_'):add(mat,'emissive','0 0 0 1')
        if collision:
            c=add(link,'collision',name=name+'_collision');add(c,'pose',pose)
            c.append(E.fromstring(E.tostring(geom)))
        return n
    for name,color in [('panels','.95 .95 .95 1'),('diamonds','.015 .015 .015 1'),
                       ('beacon_pipe','.95 .95 .95 1'),('sunhat','.01 .01 .01 1')]:
        part(name,('mesh',{'uri':f'model://robotx_2026_robobuoy/meshes/{name}.stl'}),color=color)
    # Conservative submerged/base envelope and hollow body's outer volume for contacts.
    for k in range(4):
        angle=k*math.pi/2
        x,y=.2159*math.cos(angle),.2159*math.sin(angle)
        part(f'float_{k}',('cylinder',{'radius':.0381,'length':.4318}),
             f'{x} {y} 0 1.57079632679 0 {angle}',color='.45 .12 .65 1',collision=True)
    part('top_mount',('box',{'size':'.18 .18 .03'}),'0 0 .42 0 0 0',collision=True)
    for k in range(4):
        c=add(link,'collision',name=f'panel_{k}');add(c,'pose',f'0 0 .25 0 0 {k*math.pi/2}')
        add(add(add(c,'geometry'),'box'),'size','.4318 .4318 .3048')
        break  # Conservative hull envelope; visible panels retain the official taper.
    part('light_top',('cylinder',{'radius':.039,'length':.003}),'0 0 .558 0 0 0',color='0 0 0 1')
    part('light_side',('cylinder',{'radius':.045,'length':.036}),'0 0 .49 0 0 0',color='0 0 0 1')
    part('beacon_collision',('cylinder',{'radius':.04445,'length':.13335}),
         '0 0 .501675 0 0 0',collision=True)
    # Its visual would occlude the emissive aperture; retain only the collision.
    link.remove(link.find("visual[@name='beacon_collision']"))
    E.indent(root);E.ElementTree(root).write(OUT/'model.sdf',encoding='utf-8',xml_declaration=True)
    (OUT/'model.config').write_text('<model><name>RobotX 2026 RoboBuoy</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>USV navigation</name></author><description>Official 2026 exterior, fixed mooring approximation. See docs/task1.</description></model>\n')
    print(OUT)


if __name__=='__main__':main()
